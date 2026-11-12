from __future__ import annotations
"""
avtp.py — AVTP (IEEE 1722) Protocol Library (IEC 61883 Format)
===============================================================
Implements a Scapy-based AVTP IEC 61883/IIDC layer (Subtype 0x00) and
helper functions for fragmenting/reassembling image data across AVTP packets.

Protocol reference: IEEE 1722-2016 / IEC 61883-4
EtherType: 0x22F0
"""

import struct
import time
from typing import Optional


# ── Constants ─────────────────────────────────────────────────────────────────

AVTP_ETHERTYPE = 0x22F0
# AVTP Subtype 0x00 = IEC 61883/IEC Format
AVTP_SUBTYPE = 0x00
# AVTP multicast destination MAC (IEEE 1722 Annex B)
AVTP_DST_MAC = "91:e0:f0:00:fe:00"

#CIP_HEADER = bytes([0x3F, 0x06, 0xC4, 0x60, 0xA0, 0x00, 0x00, 0x00])  # CIP Header (8 bytes)

# 1394 header: firewire header
#   - Byte 1 (5f): Tag=1, Channel=31 (Native AVTP)
#   - Byte 2 (a0): Tcode=0x0A, App-control=0x0



#bytefield= 1 bytes
#xlong = 8 bytes
#xintfield = 4 bytes
#shortfield= 2 bytes 


# sender--> camera

def fragment_mpegts_stream(
    ts_bytes: bytes,
    stream_id: int,
    seq_counter: int,
    src_mac: str,
    dst_mac: str = AVTP_DST_MAC,
    blocks_per_pkt: int = 2,
    initial_dbc: int = 0X00
) -> list[bytes]:

    """
    Agrupa blocos MPEG-TS (192 bytes cada) dentro de pacotes Ethernet/AVTP IEC 61883-4.
    """

    block_size = 192
    bytes_per_pkt = block_size * blocks_per_pkt  # 384 bytes de payload MPEG-TS
    packets = []
    
    current_seq = seq_counter & 0xFF
    current_dbc = initial_dbc & 0xFF  # Mantém o DBC dinamico (0-255)

    # Converte os MACs de string Hex para bytes puros (6 bytes cada)
    mac_dst_bytes = bytes.fromhex(dst_mac.replace(":", "").replace("-", ""))
    mac_src_bytes = bytes.fromhex(src_mac.replace(":", "").replace("-", ""))

    # 1. Cabeçalho Ethernet (14 Bytes)
    eth_hdr = struct.pack("!6s6sH", mac_dst_bytes, mac_src_bytes, AVTP_ETHERTYPE)

    # Pacote TS Nulo padrão (188 bytes + 4 bytes SPH = 192 bytes) para preenchimento de sobras
    null_ts_block = b"\x00\x00\x00\x00\x47\x1f\xff\x10" + (b"\xff" * 184)


    for i in range(0, len(ts_bytes), bytes_per_pkt):
        payload_chunk = ts_bytes[i : i + bytes_per_pkt]

        if len(payload_chunk) < bytes_per_pkt:# se menor que 384bytes ppreenche com 0
            needed_bytes = bytes_per_pkt - len(payload_chunk)
            padding = (null_ts_block * blocks_per_pkt)[:needed_bytes]
            payload_chunk = payload_chunk + padding
            
        
        header_1394 =bytes([ 0x5F, 0xA0])             # Tag + Channel + Tcode, App-control=0x0

        dynamic_cip_header = bytes([ #cip(8bytes + 2 bytes de 1394)
            0x3F, 0x06,             # QI1, SID, DBS
            0XC4,                   # Q1 FN e SPH
            current_dbc,            # Q1 DBC Dinâmico (0x00, 0x10, 0x20 ... 0xF0)
            0xA0,                   # Q2 FMT (MPEGTS), FDF
            0x00,0x00,0x00          # Q2 FDF (24 bits / 3 Bytes) -> TSF=0, Res=0
        ])


        # Payload do AVTP = CIP Header (8B) + MPEG-TS (384B) = 392 Bytes (0x0188)
        avtp_payload =  header_1394 + dynamic_cip_header +  payload_chunk
        # timestamp= time.time_ns() & 0xFFFFFFFF

        
        avtp_hdr = struct.pack(
            "!BBBBQIIH",
            AVTP_SUBTYPE,       # 1. Subtype (0x00)
            0x88,               # 2. Flags (Media Clock Restart = True) 1 Byte
            current_seq,        # 3. Sequence Number  1 Byte
            0x00,               # 4. Reserved  1 Byte
            stream_id,          # 5. Stream ID (8B)
            0,                  # 6. AVTP Timestamp (4B)
            0x00000000,         # 7. Gateway Info (4B -> 0x00000000)
            0x0188              # 8. Stream Data Length (2B -392)
        )
        
        # 3. Pacote Bruto (14B Ethernet + 22B AVTP + 8B CIP + 384B MPEG-TS + ###################
        full_packet = eth_hdr + avtp_hdr + avtp_payload
        packets.append(full_packet)
        
        
        current_seq = (current_seq + 1) & 0xFF


        # LÓGICA DO DBC: Incrementa de 16 em 16 e reseta após 240 (0xF0)
        next_dbc = current_dbc + (8 * blocks_per_pkt)  # +16 (0x10) por pacote
        if next_dbc > 240:
            current_dbc = 0  # Reinicia o ciclo em 0x00
        else:
            current_dbc = next_dbc
            

    return packets


## receiver


def locate_ts_payload(raw_pkt: bytes) -> Optional[tuple[int, int]]:
    """
    Localiza o offset do primeiro Sync Byte (0x47) do payload MPEG-TS
    dentro do pacote bruto e o tamanho de bloco correspondente (com ou sem
    os 4 bytes de SPH), testando os mesmos alinhamentos de fallback que
    parse_mpegts_stream_packet usa. Retorna (offset, block_size) ou None
    se não achar o sync byte em nenhum alinhamento esperado.

    Separado de parse_mpegts_stream_packet para que quem só precisa saber
    ONDE o payload TS está dentro do pacote bruto — por exemplo, pra
    regravar bytes corrompidos de volta no mesmo lugar, sem reconstruir o
    pacote do zero — não precise duplicar essa lógica de alinhamento.
    """
    HEADER_OFFSET = 50       # 14 Ethernet + 24 AVTP + 8 CIP + 4 SPH
    BLOCK_SIZE = 192          # 4B SPH + 188B TS

    if not isinstance(raw_pkt, bytes) or len(raw_pkt) < (HEADER_OFFSET + 188):
        return None

    if raw_pkt[HEADER_OFFSET] == 0x47:
        return HEADER_OFFSET, BLOCK_SIZE

    # Fallback de segurança para o caso de algum pacote sem o CIP (38) ou sem o SPH (46)
    if raw_pkt[38] == 0x47:
        return 38, 188
    if raw_pkt[42] == 0x47:
        return 42, 192
    if raw_pkt[46] == 0x47:
        return 46, 188

    return None  # Não encontrou o byte de sincronismo 0x47 no alinhamento esperado


def parse_mpegts_stream_packet(raw_pkt: bytes) -> Optional[bytes]:
    """
    Extrai blocos MPEG-TS de 188 bytes a partir do cabeçalho completo:
    Ethernet (14B) + AVTP (24B) + CIP (8B) + SPH (4B) = 50B até o Sync Byte 0x47.
    """
    located = locate_ts_payload(raw_pkt)
    if located is None:
        return None
    header_offset, block_size = located

    extracted_ts = bytearray()
    # Pega o primeiro bloco no offset localizado e pula de block_size em
    # block_size se houver múltiplos blocos no pacote.
    for i in range(header_offset, len(raw_pkt), block_size):
        ts_pkt = raw_pkt[i : i + 188]
        if len(ts_pkt) == 188 and ts_pkt[0] == 0x47:
            extracted_ts.extend(ts_pkt)

    return bytes(extracted_ts) if extracted_ts else None


def splice_ts_payload(raw_pkt: bytes, ts_data: bytes) -> bytes:
    """
    Regrava os blocos TS de 188 bytes dentro de uma CÓPIA do pacote bruto
    com os bytes de ts_data — usada pelo receiver.py para gravar de volta,
    no mesmo pacote de onde veio, uma corrupção aplicada sobre o ts_data já
    extraído por parse_mpegts_stream_packet(raw_pkt). ts_data deve ter o
    mesmo tamanho/mesma contagem de blocos de 188B que parse_mpegts_stream_
    packet(raw_pkt) retornaria; qualquer sobra de ts_data além dos blocos
    que cabem no pacote original é ignorada.

    Sem isso, "corromper" só alteraria uma cópia solta dos bytes TS — o
    pacote de fato encaminhado adiante continuaria limpo. Com isso, o que
    segue adiante (local e pela rede/socket) é o mesmo pacote que um
    atacante real teria adulterado, byte a byte, no lugar certo.
    """
    located = locate_ts_payload(raw_pkt)
    if located is None:
        return raw_pkt
    header_offset, block_size = located

    out = bytearray(raw_pkt)
    pos = 0
    for i in range(header_offset, len(raw_pkt), block_size):
        if pos + 188 > len(ts_data):
            break
        out[i : i + 188] = ts_data[pos : pos + 188]
        pos += 188

    return bytes(out)