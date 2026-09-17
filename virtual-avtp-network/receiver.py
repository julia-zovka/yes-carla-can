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

from scapy.all import sniff
import avtp as avtp_lib


# ── Argument parsing ──────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(description="AVTP MPEG-TS Video Receiver with FFplay")
    parser.add_argument("-i", "--interface", default="veth-r", help="Network interface to listen on (default: veth-r)")
    parser.add_argument("-o", "--output-dir", default=None, help="Directory to save received frames as PNG files (optional)")
    parser.add_argument("-t", "--timeout", type=float, default=None, help="Stop sniffing after this many seconds of inactivity (optional)")
    return parser.parse_args()


# ── Receiver state ────────────────────────────────────────────────────────────

class ReceiverState:
    """
    Acumula os blocos MPEG-TS recebidos e grava diretamente em um arquivo de vídeo (.ts).
    """

    def __init__(self, output_dir: Path | None = None, output_filename: str = "output_stream.ts"):
        self.packets_received = 0
        self.bytes_received = 0
        self.start_time = time.time()

        self.last_sequence_num = None
        self.packets_lost = 0

        
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
            #"-flags", "low_delay",       # Força decodificação de baixa latência
            #"-framedrop",                # Descarta quadros atrasados para não travar a GUI
            #"-sync", "ext",              # Sincroniza pelo relógio do sistema
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


    def handle_packet(self, packet):
        # Extrai os 384 bytes de MPEG-TS usando a nova função
        ts_data = avtp_lib.parse_mpegts_stream_packet(packet)
        if ts_data is None:
            return

        self.packets_received += 1
        self.bytes_received += len(ts_data)

        # Monitoramento de Perda de Pacotes (Sequence Number) pelo pacote camad ethernet
        raw_pkt = bytes(packet)
        if len(raw_pkt) >= 17:
            current_seq = raw_pkt[16]  # Byte no índice 16 (0-indexed)
            
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

        # Gravaçao somente se a opção --output-dir for usada
        if self.out_file:
            self.out_file.write(ts_data)


        # Escreve com verificação estrita de processo ativo
        if self.ffplay_proc and self.ffplay_proc.poll() is None:
            try:
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


    stop = {"flag": False}

    def _sigint_handler(sig, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _sigint_handler)
    try:
        sniff(
            iface=args.interface,
            filter=f"ether proto {avtp_lib.AVTP_ETHERTYPE}",
            prn=state.handle_packet,
            store=0,
            stop_filter=lambda _: stop["flag"],
            timeout=args.timeout,
            rcvbuf=2 * 1024 * 1024  # 2 MB de buffer de recepção
        )
    except KeyboardInterrupt:
        pass

    finally:
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
