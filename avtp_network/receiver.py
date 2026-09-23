#!/usr/bin/env python3
"""
receiver.py — AVTP MPEG-TS Receiver (Live FFplay Display)
=========================================================
Recebe pacotes AVTP MPEG-TS da rede e os envia em tempo real
para o 'ffplay' através de um sub-processo via PIPE (stdin).

Options
-------
    --interface     Network interface to listen on     (default: veth-r)
    --output-dir    Directory to save received frames  (optional)
    --timeout       Stop after N seconds of silence    (default: off)
"""

import argparse
import signal
import time
import subprocess  
from pathlib import Path
import socket
import sys

from scapy.all import sniff
import avtp as avtp_lib


# ── Argument parsing ──────────────────────────────────────────────────────────
AVTP_ETHERTYPE = 0x22F0  # EtherType do IEEE 1722 / AVTP

def parse_args():
    parser = argparse.ArgumentParser(description="AVTP MPEG-TS Video Receiver with FFplay")
    parser.add_argument("-i", "--interface", default="veth-r", help="Network interface to listen on (default: veth-r)")
    parser.add_argument("-o", "--output-dir", default=None, help="Directory to save received frames as PNG files (optional)")
    parser.add_argument("-t", "--timeout", type=float, default=None, help="Stop sniffing after this many seconds of inactivity (optional)")
    return parser.parse_args()

def parse_mpegts_stream_packet(raw_pkt: bytes) -> bytes | None:
    """
    Extrai e limpa os 2 blocos MPEG-TS (188 bytes cada = 376 bytes) contidos no 
    pacote AVTP nativo, removendo Ethernet, AVTP, 1394, CIP e o cabeçalho SPH 
    (5 bytes) de cada bloco.

    Mapeamento de Offsets:
      - 00..13 (14B): Ethernet Header
      - 14..35 (22B): AVTP Header (Sequence Number no index 16)
      - 36..37 ( 2B): 1394 Header
      - 38..45 ( 8B): CIP Header (IEC 61883-4)
      - 46..50 ( 5B): SPH Bloco 1 -> Offset 51: Byte Sync 0x47 (188B TS)
      - 238..242 (5B): SPH Bloco 2 -> Offset 243: Byte Sync 0x47 (188B TS)
    """
    # Tamanho mínimo do pacote de rede: 14 + 22 + 2 + 8 + (2 * 192) = 430 bytes
    if len(raw_pkt) < 430:
        return None

    # Offset base após Ethernet + AVTP + 1394 + CIP = 46 bytes
    BASE_PAYLOAD_OFFSET = 46

    # Bloco 1 (192B): Pula 5B de SPH (46..50) -> pega 188B TS (51..238)
    block1 = raw_pkt[BASE_PAYLOAD_OFFSET + 5 : BASE_PAYLOAD_OFFSET + 193]

    # Bloco 2 (192B): Pula 5B de SPH (238..242) -> pega 188B TS (243..430)
    block2 = raw_pkt[BASE_PAYLOAD_OFFSET + 197 : BASE_PAYLOAD_OFFSET + 385]

    # Valida se ambos os blocos começam rigorosamente com o Sync Byte MPEG-TS (0x47)
    if block1.startswith(b"\x47") and block2.startswith(b"\x47"):
        return block1 + block2  # Retorna exatamente 376 bytes

    # Fallback: Se apenas o primeiro bloco for válido
    if block1.startswith(b"\x47") and len(block1) == 188:
        return block1

    return None

# ── Receiver state ────────────────────────────────────────────────────────────

class ReceiverState:
    """
    Acumula os blocos MPEG-TS recebidos -> jitter buffer-> envia pro ffplay e pro discom arquivo de vídeo (.ts).
    """

    def __init__(self, output_dir: Path | None = None, output_filename: str = "output_stream.ts"):
        self.packets_received = 0
        self.bytes_received = 0
        self.start_time = time.time()

        self.last_sequence_num = None
        self.packets_lost = 0

        #IEC 61883-4 Annex A De-Jitter Buffer (3264 bytes)
        self.jitter_buffer = bytearray()
        self.is_buffered = False
        self.JITTER_BUFFER_SIZE = 3264
        
        self.out_file = None
        if output_dir:
            output_path = output_dir / "output_stream.ts"
            self.out_file = open(output_path, "wb")
            print(f"  [i] Cópia em disco sendo salva em: {output_path.resolve()}")
        
        
        # Comando ffplay lendo diretamente da entrada padrão (pipe:0)
        ffplay_cmd = [
            "ffplay",
            "-window_title", "AVTP Live Stream (FFplay)",
            "-probesize", "32000",          # Reduz latência inicial de leitura
            "-analyzeduration", "0",        # Inicia a reprodução imediatamente
            "-fflags", "nobuffer",          # Minimiza o atraso (buffering) do vídeo ao vivo
            "-flags", "low_delay",       # Força decodificação de baixa latência
            "-framedrop",                # Descarta quadros atrasados para não travar a GUI
            "-sync", "ext",              # Sincroniza pelo relógio do sistema
            "-f", "mpegts",                 # Força o demuxer MPEG-TS
            "-i", "pipe:0"                  # Lê do PIPE recebido do Python
        ]

        # Abre o processo FFplay
        self.ffplay_proc = subprocess.Popen(
            ffplay_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            bufsize=0
        )

        # Pequena pausa para verificar se o ffplay não morreu na largada
        time.sleep(0.2)
        if self.ffplay_proc.poll() is not None:
            _, err = self.ffplay_proc.communicate()
            print("\n  [!] ERRO ao iniciar o ffplay:")
            print(err.decode("utf-8", errors="replace"))


    def handle_packet(self, raw_pkt: bytes):
        if len(raw_pkt) < 430:
            return

        # 1. Checagem de Perda de Pacotes pelo Sequence Number (Byte index 16)
        current_seq = raw_pkt[16]
        if self.last_sequence_num is not None:
            expected_seq = (self.last_sequence_num + 1) & 0xFF
            if current_seq != expected_seq:
                lost = (current_seq - expected_seq) & 0xFF
                self.packets_lost += lost
                print(
                    f"  [!] PERDA DETECTADA: Esperado {expected_seq:>3} "
                    f"| Recebido {current_seq:>3} | Perdidos neste salto: {lost}"
                )
        self.last_sequence_num = current_seq

       # 2. Extração e limpeza do payload MPEG-TS (376B por pacote válido)
        ts_data = parse_mpegts_stream_packet(raw_pkt)
        if ts_data is None:
            return  # Descarta caso não passe na validação de sincronismo 0x47

        self.packets_received += 1
        self.bytes_received += len(ts_data)

        # Gravaçao somente se a opção --output-dir for usada
        if self.out_file:
            self.out_file.write(ts_data)


        # Escreve com verificação estrita de processo ativo
        if self.ffplay_proc and self.ffplay_proc.poll() is None:
            try:
                if not self.is_buffered:
                    self.jitter_buffer.extend(ts_data)
                    if len(self.jitter_buffer) >= self.JITTER_BUFFER_SIZE:
                        self.ffplay_proc.stdin.write(self.jitter_buffer)
                        self.ffplay_proc.stdin.flush()
                        self.is_buffered = True
                        self.jitter_buffer.clear()
                else:
                    self.ffplay_proc.stdin.write(ts_data)
                    self.ffplay_proc.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
                
        
        # Log a cada 100 pacotes recebidos
        if self.packets_received % 100 == 0:
            elapsed = time.time() - self.start_time
            rate_kbps = (self.bytes_received * 8 / 1000) / elapsed if elapsed > 0 else 0
            print(
                f"  [RX TS] Pacotes: {self.packets_received:>6}"
                f" | Total: {self.bytes_received / 1024:>7.1f} KB"
                f" | Taxa: {rate_kbps:>6.1f} kbps"
                f" | Perdas: {self.packets_lost:>4}"
            )

    def close(self):
        """Fecha o arquivo ao encerrar o script."""
        if self.out_file:
            self.out_file.close()

        if hasattr(self, "ffplay_proc") and self.ffplay_proc:
            try:
                if self.ffplay_proc.stdin:
                    self.ffplay_proc.stdin.close()
                self.ffplay_proc.terminate()
                self.ffplay_proc.wait(timeout=1)
            except Exception:
                pass


# ── Main ──────────────────────────────────────────────────────────────────────

def run(args):
    
    output_dir = None
    if args.output_dir:
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

    state = ReceiverState(output_dir=output_dir)

    print("=" * 65)
    print("  AVTP Video Receiver (MPEG-TS Stream)")
    print("=" * 65)
    print(f"  Interface : {args.interface}")
    print(f"  Output dir: {args.output_dir or '(diretório atual)'}")
    print(f"  Timeout   : {args.timeout or 'none'}")
    print("=" * 65)
    print("  Listening for AVTP MPEG-TS stream... (Ctrl-C to quit)")
    print()


   # Criar Socket RAW nativo do Linux focado no EtherType AVTP (0x22F0)
    try:
        sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(AVTP_ETHERTYPE))
        sock.bind((args.interface, socket.htons(AVTP_ETHERTYPE)))
        # Expande o buffer de recepção no kernel para 16MB para evitar perdas
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
    except Exception as e:
        print(f"  [!] Erro ao abrir socket RAW na interface '{args.interface}': {e}")
        sys.exit(1)

    if args.timeout:
        sock.settimeout(args.timeout)

    stop_requested = False



    def _sigint_handler(sig, frame):
        nonlocal stop_requested
        stop_requested = True


    signal.signal(signal.SIGINT, _sigint_handler)

    print("  Listening for AVTP MPEG-TS stream... (Ctrl-C to quit)\n")

    try:
        while not stop_requested:
            try:
                raw_pkt, _ = sock.recvfrom(2048)
                state.handle_packet(raw_pkt)
            except socket.timeout:
                print("\n  [i] Timeout atingido sem pacotes. Encerrando recepção.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        state.close()


    elapsed = time.time() - state.start_time
    avg_rate = (state.bytes_received * 8 / 1000) / elapsed if elapsed > 0 else 0

    print("\n")
    print("=" * 65)
    print("  Reception Summary")
    print("=" * 65)
    print(f"  Packets received : {state.packets_received}")
    print(f"  Bytes received   : {state.bytes_received:,} B")
    print(f"  Elapsed time     : {elapsed:.2f} s")
    print(f"  Average Rate     : {avg_rate:.1f} kbps")
    print("=" * 65)

  
if __name__ == "__main__":
    args = parse_args()
    run(args)

##rcvbuf 
# Atua no nível da placa de rede / sistema operacional. Ele evita que o Kernel do Linux descarte pacotes chegados na interface caso a aplicação Python demore alguns milissegundos para processar o loop da função sniff().



##Jitter Buffer de 3264B
#Atua na lógica do seu código antes de entregar a mídia ao ffplay. Ele garante que o decodificador de vídeo não sofra com underrun
