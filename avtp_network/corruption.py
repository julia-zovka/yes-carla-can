#!/usr/bin/env python3
"""
corruption_test.py — Receiver-side Robustness / Fault-Injection Harness
=========================================================================
Ferramenta de teste de robustez para o pipeline de decodificação do
receiver.py. NÃO opera na rede: atua *depois* que o pacote já foi
legitimamente recebido e parseado pelo seu próprio receiver, corrompendo
deliberadamente os bytes do bloco MPEG-TS antes de repassá-los ao ffplay.

Objetivo: medir empiricamente o comportamento do ffplay/decoder H.264
quando um bloco correspondente a um I-frame chega corrompido, sem
precisar de nenhum vetor de injeção em trânsito.

Uso: importado e ativado opcionalmente pelo receiver.py via --corrupt-test.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum


class CorruptionMode(Enum):
    SYNC_BYTE = "syncbyte"   # flips o Sync Byte (0x47) do primeiro pacote TS do bloco
    ZERO_FILL = "zero"       # zera os bytes do payload do bloco inteiro
    BIT_FLIP = "bitflip"     # inverte um único bit em posição fixa (corrupção mínima)


@dataclass
class CorruptionEvent:
    index: int              # número sequencial do evento de corrupção
    packet_count: int       # contagem de pacotes handle_packet() no momento
    mode: str
    timestamp: float
    bytes_affected: int


@dataclass
class CorruptionTester:
    """
    Conta blocos TS recebidos e, a cada `period` blocos (aproximando o
    ciclo de GOP configurado no encoder — ver `g` em camera.py), corrompe
    deliberadamente UM bloco antes de ele seguir para o ffplay/disco.

    period=5 replica o GOP padrão usado no MPEGTSStreamEncoder (g=5).
    Ajuste conforme o GOP real do seu encoder se ele mudar.
    """
    period: int = 5
    mode: CorruptionMode = CorruptionMode.SYNC_BYTE
    enabled: bool = True

    _ts_chunk_count: int = field(default=0, init=False)
    _event_count: int = field(default=0, init=False)
    events: list[CorruptionEvent] = field(default_factory=list, init=False)

    def maybe_corrupt(self, ts_data: bytes) -> tuple[bytes, bool]:
        """
        Recebe um bloco ts_data já legitimamente extraído pelo
        parse_mpegts_stream_packet() do seu próprio receiver.
        Retorna (dados_possivelmente_corrompidos, foi_corrompido).
        """
        if not self.enabled or not ts_data:
            return ts_data, False

        self._ts_chunk_count += 1

        # Dispara no bloco que, pela contagem, cai no início do ciclo de GOP
        if self._ts_chunk_count % self.period != 0:
            return ts_data, False

        corrupted = bytearray(ts_data)

        if self.mode == CorruptionMode.SYNC_BYTE:
            # Bloco TS = 188 bytes; sync byte 0x47 é o primeiro byte de
            # CADA pacote TS de 188B dentro do bloco recebido.
            for off in range(0, len(corrupted), 188):
                corrupted[off] = 0x00  # invalida o sync byte
            affected = len(corrupted) // 188

        elif self.mode == CorruptionMode.ZERO_FILL:
            corrupted[:] = b"\x00" * len(corrupted)
            affected = len(corrupted)

        elif self.mode == CorruptionMode.BIT_FLIP:
            # Corrupção mínima: 1 bit em posição fixa próxima ao início
            # do payload (dentro da área do NAL, não do header TS).
            flip_pos = min(5, len(corrupted) - 1)
            corrupted[flip_pos] ^= 0b00000001
            affected = 1

        else:
            return ts_data, False

        self._event_count += 1
        evt = CorruptionEvent(
            index=self._event_count,
            packet_count=self._ts_chunk_count,
            mode=self.mode.value,
            timestamp=time.time(),
            bytes_affected=affected,
        )
        self.events.append(evt)

        print(
            f"  [CORRUPT-TEST] Evento #{evt.index} | modo={evt.mode} "
            f"| bloco TS nº{evt.packet_count} | bytes afetados={evt.bytes_affected}"
        )

        return bytes(corrupted), True

    def summary(self) -> str:
        lines = [
            "=" * 65,
            "  Corruption Test Summary",
            "=" * 65,
            f"  Modo              : {self.mode.value}",
            f"  Período (GOP)     : {self.period}",
            f"  Blocos TS totais  : {self._ts_chunk_count}",
            f"  Eventos injetados : {self._event_count}",
        ]
        if self.events:
            lines.append("  Timeline:")
            for e in self.events:
                lines.append(
                    f"    #{e.index:>3}  bloco={e.packet_count:>6}  "
                    f"bytes={e.bytes_affected:>4}  t={e.timestamp:.3f}"
                )
        lines.append("=" * 65)
        return "\n".join(lines)