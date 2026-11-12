#!/usr/bin/env python3
"""
corruption_test.py — Receiver-side Robustness / Fault-Injection Harness
=========================================================================
Ferramenta de teste de robustez para o pipeline de decodificação do
receiver.py. NÃO opera na rede: atua *depois* que o pacote já foi
legitimamente recebido e parseado pelo seu próprio receiver, corrompendo
deliberadamente os bytes do bloco MPEG-TS antes de repassá-los ao ffplay.

Detecção por LIMIAR ABSOLUTO de tamanho de rajada (burst), calibrado
com dados empíricos reais de TX (ver logs de [TX MPEG-TS]):
 
    I-frame  (keyframe, g=5) : ~250-300+ pacotes AVTP por frame
    P-frame  (delta)         : ~19-93   pacotes AVTP por frame

Delimitação de frame: pacotes do MESMO frame chegam em rajada apertada
(laço de socket.send sem sleep); o próximo frame só começa após o
intervalo do sensor_tick da câmera (~33ms a 30fps).
"""
from __future__ import annotations

import time
from collections import deque
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
    burst_index: int         # número da rajada (frame) em que ocorreu
    burst_pkt_pos: int       # posição do pacote dentro da rajada atual
    mode: str
    timestamp: float
    bytes_affected: int


@dataclass
class CorruptionTester:
    """
    Detecta rajadas (frames) por gap de tempo entre chegadas de pacotes
    AVTP e corrompe pacotes de rajadas cujo tamanho, em tempo real, já
    ultrapassou `iframe_pkt_threshold` — ou seja, "isto já tem pacotes
    demais pra ser um P-frame, deve ser o I-frame".

    gap_threshold : float
        Intervalo (s) entre chegadas de pacotes que sinaliza início de
        uma nova rajada/frame. 4ms é seguro para 30fps em veth/loopback
        (o espaçamento intra-frame é sub-ms; o espaçamento entre frames
        gira em torno de ~33ms).
    iframe_pkt_threshold : int
    corrupt_whole_burst : bool
        Se True, corrompe todo pacote da rajada a partir do momento em
        que o limiar é cruzado (mais destrutivo — ataca o resto do
        I-frame). Se False, corrompe só o pacote que cruzou o limiar.
    warmup_bursts : int
        Rajadas iniciais ignoradas para o gatilho, pode ser útil pular 
        o primeiro I-frame do stream se não quiser corromper logo de cara).

    """
    gap_threshold: float = 0.004
    iframe_pkt_threshold: int = 150
    corrupt_whole_burst: bool = True
    warmup_bursts: int = 30
    mode: CorruptionMode = CorruptionMode.SYNC_BYTE
    enabled: bool = True
 
    _last_arrival: float | None = field(default=None, init=False)
    _burst_sizes: list[int] = field(default_factory=list, init=False)
    _burst_index: int = field(default=0, init=False)
    _current_burst_pkts: int = field(default=0, init=False)
    _current_burst_flagged: bool = field(default=False, init=False)
    _packet_count: int = field(default=0, init=False)
    _event_count: int = field(default=0, init=False)
    events: list[CorruptionEvent] = field(default_factory=list, init=False)

    def _apply_corruption(self, ts_data: bytes) -> tuple[bytes, int]:
        corrupted = bytearray(ts_data)
        if self.mode == CorruptionMode.SYNC_BYTE:
            for off in range(0, len(corrupted), 188):
                corrupted[off] = 0x00
            affected = max(1, len(corrupted) // 188)
        elif self.mode == CorruptionMode.ZERO_FILL:
            corrupted[:] = b"\x00" * len(corrupted)
            affected = len(corrupted)
        elif self.mode == CorruptionMode.BIT_FLIP:
            flip_pos = min(5, len(corrupted) - 1)
            corrupted[flip_pos] ^= 0b00000001
            affected = 1
        else:
            return ts_data, 0
        return bytes(corrupted), affected
 




    def maybe_corrupt(self, ts_data: bytes) -> tuple[bytes, bool]:
        """
        Recebe um bloco ts_data já legitimamente extraído pelo
        parse_mpegts_stream_packet() do seu próprio receiver.
        Retorna (dados_possivelmente_corrompidos, foi_corrompido).
        """
        if not self.enabled or not ts_data:
            return ts_data, False

        now = time.time()
        self._packet_count += 1


        # ── Delimitação de rajada por gap de tempo ─────────────────────────
        is_new_burst = (
            self._last_arrival is not None
            and (now - self._last_arrival) > self.gap_threshold
        )
        if is_new_burst and self._current_burst_pkts > 0:
            self._burst_sizes.append(self._current_burst_pkts)
            self._burst_index += 1
            self._current_burst_pkts = 0
            self._current_burst_flagged = False
 
        self._last_arrival = now
        self._current_burst_pkts += 1
 
        if not self.enabled:
            return ts_data, False
 
        # ── Gatilho por limiar absoluto: a rajada já é "grande demais" ─────
        #    pra ser um P-frame, então deve ser o I-frame.
        burst_is_candidate = (
            self._burst_index >= self.warmup_bursts
            and self._current_burst_pkts >= self.iframe_pkt_threshold
        )
 
        should_corrupt_this_pkt = False
        if burst_is_candidate:
            if not self._current_burst_flagged:
                print(
                    f"  [CORRUPT-TEST] Rajada #{self._burst_index} atingiu "
                    f"{self._current_burst_pkts} pacotes (limiar={self.iframe_pkt_threshold}) "
                    f"— classificada como I-FRAME, iniciando corrupção"
                )
            self._current_burst_flagged = True
            should_corrupt_this_pkt = self.corrupt_whole_burst or (
                self._current_burst_pkts == self.iframe_pkt_threshold
            )
 
        if not should_corrupt_this_pkt:
            return ts_data, False
 
        ts_data, affected = self._apply_corruption(ts_data)
        self._event_count += 1
        evt = CorruptionEvent(
            index=self._event_count,
            packet_count=self._packet_count,
            burst_index=self._burst_index,
            burst_pkt_pos=self._current_burst_pkts,
            mode=self.mode.value,
            timestamp=now,
            bytes_affected=affected,
        )
        self.events.append(evt)
 
        return ts_data, True

    def summary(self) -> str:
        # fecha a última rajada em andamento para fins de contagem no resumo
        if self._current_burst_pkts > 0:
            self._burst_sizes.append(self._current_burst_pkts)
 
        lines = [
            "=" * 65,
            "  Corruption Test Summary (limiar absoluto de tamanho de rajada)",
            "=" * 65,
            f"  Modo                    : {self.mode.value}",
            f"  gap_threshold           : {self.gap_threshold * 1000:.1f} ms",
            f"  iframe_pkt_threshold    : {self.iframe_pkt_threshold} pacotes",
            f"  corrupt_whole_burst     : {self.corrupt_whole_burst}",
            f"  Pacotes AVTP totais     : {self._packet_count}",
            f"  Rajadas (frames) vistas : {len(self._burst_sizes)}",
            f"  Eventos injetados       : {self._event_count}",
        ]
        if self._burst_sizes:
            classified_i = sum(1 for s in self._burst_sizes if s >= self.iframe_pkt_threshold)
            classified_p = len(self._burst_sizes) - classified_i
            lines.append(
                f"  Rajadas classificadas   : {classified_i} como I-frame, "
                f"{classified_p} como P-frame"
            )
            lines.append(
                f"  Tamanhos observados     : min={min(self._burst_sizes)}  "
                f"max={max(self._burst_sizes)}  "
                f"média={sum(self._burst_sizes)/len(self._burst_sizes):.1f}"
            )
        if self.events:
            # Agrupa os eventos por rajada em vez de listar cada pacote
            # individual (com corrupt_whole_burst=True, um único I-frame
            # pode gerar centenas de eventos — um por rajada é suficiente
            # para o relatório).
            by_burst: dict[int, list[CorruptionEvent]] = {}
            for e in self.events:
                by_burst.setdefault(e.burst_index, []).append(e)
 
            lines.append("  I-frames atacados:")
            for burst_idx in sorted(by_burst):
                evs = by_burst[burst_idx]
                first, last = evs[0], evs[-1]
                lines.append(
                    f"    rajada={burst_idx:>4}  "
                    f"pacotes_corrompidos={len(evs):>4} "
                    f"(posições {first.burst_pkt_pos}-{last.burst_pkt_pos} na rajada)  "
                    f"pacote_global={first.packet_count}-{last.packet_count}  "
                )
        lines.append("=" * 65)
        return "\n".join(lines)