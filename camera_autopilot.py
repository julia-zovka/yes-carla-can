#!/usr/bin/env python3
"""
camera_autopilot.py — Autopilot baseado na imagem RECEBIDA (não no mundo)
==========================================================================
Processo independente, roda no namespace de rede RAIZ (tem acesso ao
vcan0). Substitui o Traffic Manager nativo do CARLA: em vez de decidir com
estado privilegiado da simulação, decide com base na MESMA imagem que o
receiver.py recebeu via AVTP — incluindo qualquer corrupção de teste
aplicada em corruption.py.

Pipeline completo:

    receiver.py [netns "receiver"]
        recebe pacotes AVTP brutos, (opcional) corrompe e regrava no pacote
            │  socket Unix, pacotes brutos com framing (4B tamanho + pacote)
            │  (atravessa namespace de rede sem problema — namespace de
            │  rede não isola filesystem/sockets Unix)
            ▼
    camera_autopilot.py [netns raiz]  ←── você está aqui
        desempacota o AVTP do zero (parse_mpegts_stream_packet) → ts_data
        decodifica ts_data (PyAV) → frames
        detecção de faixa (OpenCV) → steer / throttle
        máquina de estados de colisão (lê CAN, já publicado por
        sensors/collision.py) → RECOVER / REORIENT
        manda THROTTLE/STEER/BRAKE/REVERSE via CAN (vcan0)
            │
            ▼
    CARLA_client_module.py / gui/keyboard_control.py
        can_network.recv_msg() decodifica de volta → apply_control()
        (zero mudança necessária aqui — já decodifica qualquer sender)

Liga/desliga pela tecla 'K' na janela do CARLA_client_module.py, que manda
a mensagem CAN CAM_AUTOPILOT_ENABLE. Enquanto desligado, este processo só
decodifica os frames (pra manter o estado de visão "aquecido") mas não
manda nada no barramento.

Uso:
    PYTHONPATH=. "$CONDA_PREFIX/bin/python" camera_autopilot.py \\
        --socket /tmp/yes_carla_can_cam.sock --vcan vcan0

O receiver.py precisa ser lançado com:
    ... avtp_network/receiver.py -i veth-r -o . \\
        --cam-autopilot-socket /tmp/yes_carla_can_cam.sock
"""
from __future__ import annotations

import argparse
import math
import os
import queue
import socket
import struct
import threading
import time
import traceback

import numpy as np
import cv2
import av
import carla

from avtp_network.avtp import parse_mpegts_stream_packet
from can_network.network import CAN_Network, VCAN_CHANNEL


# ── Parâmetros da máquina de estados de recuperação de colisão ────────────
RECOVER_DURATION = 2.5      # segundos em ré, se afastando do obstáculo
RECOVER_THROTTLE = 0.65     # ré mais decidida (0.5 era fraco demais pra sair do obstáculo)
MAX_RECOVER_TOTAL = 6.0     # teto absoluto de ré, mesmo estendendo — evita loop infinito se
                             # ficar preso num canto/parede (ré não consegue de fato se afastar)
MIN_PERSIST_INTENSITY = 5.0 # abaixo disso, colisão "persistindo" é tratada como ruído do sensor
REORIENT_DURATION = 1.5     # segundos parado, endireitando a direção
DRIVE_THROTTLE_DEFAULT = 0.35
NO_LANE_THROTTLE = 0.15     # quando nenhuma faixa é detectada: desacelera, não inventa direção
CONTROL_HZ = 10.0           # frequência de envio CAN

# ── Parâmetros do processamento de imagem (detecção de faixa) ─────────────
STEER_SMOOTHING = 0.6       # peso do steer ANTERIOR na suavização (0 = sem suavização,
                             # mais perto de 1 = mais suave/lento pra reagir)
MODE_SWITCH_FRAMES = 5      # frames CONSECUTIVOS discordando do modo ativo (LANE/NO_LANE)
                             # antes de trocar de fato — debounce contra flicker frame-a-frame
                             # da detecção (ver comentário na histerese dentro de _lane_detect)

# ── Parâmetros do fallback de waypoint (só usado quando NENHUMA faixa é
# visível — ver _lane_detect/_update_waypoint_steer). A imagem da câmera
# continua sendo o sinal DOMINANTE sempre que há faixa visível; isso aqui só
# resolve o caso sem sinal visual nenhum (cruzamentos, faixa desaparecida),
# onde antes o carro só repetia o último steer e seguia reto até saltar a pista.
WAYPOINT_LOOKAHEAD_METERS = 5.0   # distância do próximo waypoint consultado
WAYPOINT_STEER_DEG_RANGE = 45.0   # diferença de ângulo (graus) que já satura o steer em ±1.0

STUCK_CHECK_WINDOW = 6.0    # segundos de janela pra considerar "não andou"
STUCK_MIN_DISTANCE = 0.5    # metros mínimos de deslocamento na janela pra não contar como preso

RPC_POLL_DIVIDER = 3        # _update_waypoint_steer/_check_stuck rodam a cada N ciclos do
                             # laço de controle, não todo ciclo (menos tráfego RPC)

SIDE_PERSIST_FRAMES = 5     # frames seguidos sem detectar um lado em que ainda reusa a
                             # última posição conhecida DAQUELE lado

MAX_RAW_PACKET_SIZE = 65536 # tamanho máx. de um pacote AVTP bruto framed vindo de receiver.py


def parse_args():
    parser = argparse.ArgumentParser(description="Autopilot baseado em imagem recebida via AVTP")
    parser.add_argument("--socket", default="/tmp/yes_carla_can_cam.sock",
                         help="Caminho do socket Unix onde este processo escuta o receiver.py")
    parser.add_argument("--dbc", default="data/carla.dbc", help="Caminho do arquivo DBC")
    parser.add_argument("--vcan", default=VCAN_CHANNEL, help=f"Interface CAN (default: {VCAN_CHANNEL})")
    parser.add_argument("--show-debug", action="store_true",
                         help="Escreve o frame de detecção de faixa num arquivo .jpg continuamente "
                              "(ver --debug-image-path), pra abrir num visualizador de imagem comum")
    parser.add_argument("--debug-image-path", default="/tmp/camera_autopilot_debug.jpg",
                         help="Caminho do arquivo .jpg de debug (default: /tmp/camera_autopilot_debug.jpg)")
    parser.add_argument("--host", default="127.0.0.1",
                         help="IP do servidor CARLA, só para consulta de waypoints (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=2000,
                         help="Porta do servidor CARLA, só para consulta de waypoints (default: 2000)")
    parser.add_argument("--rolename", default="hero",
                         help="role_name do veículo ego a localizar no mundo CARLA (default: hero)")
    return parser.parse_args()


class CameraAutopilot:
    def __init__(self, args):
        self.socket_path = args.socket
        self.show_debug = args.show_debug
        self.debug_image_path = args.debug_image_path
        self.running = True

        self._debug_queue = queue.Queue(maxsize=1) if self.show_debug else None

        self.can_net = CAN_Network(dbc_path=args.dbc, channel=args.vcan)

        self.carla_host = args.host
        self.carla_port = args.port
        self.carla_rolename = args.rolename
        self._carla_client = None
        self._carla_world = None
        self._carla_map = None
        self._ego_vehicle = None
        self._waypoint_steer = None
        self._waypoint_lock = threading.Lock()
        self._wp_connected_logged = False

        self._last_mode = None

        # Histerese de modo ("debounce") — ver comentário em _lane_detect.
        self._active_mode = None
        self._pending_mode = None
        self._pending_count = 0

        # Persistência por lado (esquerda/direita) da detecção de faixa —
        # ver SIDE_PERSIST_FRAMES. Usado só na thread de decode.
        self._last_left_x = None
        self._last_right_x = None
        self._left_missing_frames = 0
        self._right_missing_frames = 0

        # Estado de percepção, atualizado pela thread de decodificação,
        # consumido pelo laço de controle.
        self._lock = threading.Lock()
        self.steer = 0.0
        self.throttle = DRIVE_THROTTLE_DEFAULT
        self.lanes_visible = False

        # Máquina de estados de colisão: DRIVE -> RECOVER -> REORIENT -> DRIVE
        self.state = "DRIVE"
        self.state_entered_at = time.time()
        self._last_collision_count = 0
        self._was_enabled = False
        # Marca quando entrou em RECOVER pela primeira vez (NÃO é resetado
        # a cada extensão, ao contrário de state_entered_at) — usado pra
        # limitar o tempo TOTAL de ré, ver comentário em
        # _update_collision_state.
        self._recover_started_at = None
        self._recover_from_stuck = False  # True quando o RECOVER atual veio do
                                           # _check_stuck (sem direção de colisão
                                           # conhecida) — nesse caso dá ré reto em
                                           # vez do esterço fixo usado pra colisão

        self._stuck_ref_location = None
        self._stuck_ref_time = None

        self._control_tick = 0

    # ------------------------------------------------------------------
    # Percepção: decodificação do stream + detecção de faixa
    # ------------------------------------------------------------------

    def _lane_detect(self, img: np.ndarray) -> tuple[float, float, bool]:
        """Detecção de faixa (máscara de cor + Canny + Hough). Retorna (steer, throttle, faixas_visiveis)."""
        h, w = img.shape[:2]

        # 1) Máscara de cor: faixas de pista são tipicamente BRANCAS ou
        # AMARELAS. Isolando essas cores antes do Canny, a maior fonte de
        # "confusão" some: sem isso, o Canny pega QUALQUER borda dentro da
        # ROI — sombra, meio-fio, junta do asfalto, pára-choque de outro
        # carro — e o Hough tenta enxergar faixa onde não tem. HLS separa
        # luminância (branco = alta) de matiz (amarelo = faixa estreita),
        # funcionando melhor que BGR/HSV sob sombra/iluminação variável.
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        white_mask = cv2.inRange(hls, (0, 190, 0), (255, 255, 255))
        yellow_mask = cv2.inRange(hls, (15, 30, 60), (35, 204, 255))
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        # Só sobrevive borda que também cai dentro da máscara de cor.
        edges = cv2.bitwise_and(edges, color_mask)

        # Região de interesse: trapézio na metade inferior da imagem, onde
        # a pista aparece — ignora céu, horizonte e o capô do carro.
        mask = np.zeros_like(edges)
        roi = np.array([[
            (int(0.05 * w), h),
            (int(0.40 * w), int(0.55 * h)),
            (int(0.60 * w), int(0.55 * h)),
            (int(0.95 * w), h),
        ]], dtype=np.int32)
        cv2.fillPoly(mask, roi, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=25,
                                 minLineLength=20, maxLineGap=100)

        left_xs, right_xs = [], []
        lookahead_y = int(0.65 * h)
        image_center = w / 2.0

        if lines is not None:
            # reshape(-1, 4): HoughLinesP varia entre (N, 1, 4) e (N, 4)
            # conforme a versão do OpenCV.
            for x1, y1, x2, y2 in lines.reshape(-1, 4):
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < 0.3:  # quase horizontal — não é borda de faixa
                    continue
                x_at_lookahead = x1 + (lookahead_y - y1) / slope
                if slope < 0 and x_at_lookahead < image_center:
                    left_xs.append(x_at_lookahead)
                elif slope > 0 and x_at_lookahead > image_center:
                    right_xs.append(x_at_lookahead)

        # Persistência por lado: uma faixa tracejada "pisca" entre frames —
        # reusa a última posição conhecida do lado por até SIDE_PERSIST_FRAMES
        # frames, em vez de já trocar de fórmula (média dos dois lados ↔ um
        # lado + meia-largura assumida).
        if left_xs:
            left_x = float(np.median(left_xs))
            self._last_left_x = left_x
            self._left_missing_frames = 0
        else:
            self._left_missing_frames += 1
            left_x = self._last_left_x if self._left_missing_frames <= SIDE_PERSIST_FRAMES else None

        if right_xs:
            right_x = float(np.median(right_xs))
            self._last_right_x = right_x
            self._right_missing_frames = 0
        else:
            self._right_missing_frames += 1
            right_x = self._last_right_x if self._right_missing_frames <= SIDE_PERSIST_FRAMES else None

        raw_visible = left_x is not None or right_x is not None

        if left_x is not None and right_x is not None:
            # Mediana em vez de média (já aplicada acima, por lado): uma
            # linha isolada e errada (outlier) não desloca o centro da
            # faixa inteiro, só a média bruta faria isso.
            lane_center = (left_x + right_x) / 2.0
            lane_throttle = DRIVE_THROTTLE_DEFAULT
        elif left_x is not None or right_x is not None:
            # só uma faixa visível (ou persistida): assume meia-largura de
            # pista a partir dela
            known = left_x if left_x is not None else right_x
            half_lane = w * 0.18
            lane_center = known + half_lane if left_x is not None else known - half_lane
            lane_throttle = DRIVE_THROTTLE_DEFAULT * 0.7
        else:
            lane_center = None
            lane_throttle = None

        # ── Histerese de modo ("debounce") ─────────────────────────────
        # Só troca de fonte de direção (LANE/WAYPOINT) depois de
        # MODE_SWITCH_FRAMES frames CONSECUTIVOS no novo estado — evita
        # salto brusco de steer trocando de fonte a cada ruído de 1-2 frames.
        raw_mode = "LANE" if raw_visible else "NO_LANE"
        if self._active_mode is None:
            self._active_mode = raw_mode
            self._pending_mode = raw_mode
            self._pending_count = 0
        elif raw_mode == self._active_mode:
            self._pending_mode = raw_mode
            self._pending_count = 0
        else:
            if raw_mode == self._pending_mode:
                self._pending_count += 1
            else:
                self._pending_mode = raw_mode
                self._pending_count = 1
            if self._pending_count >= MODE_SWITCH_FRAMES:
                self._active_mode = raw_mode
                self._pending_count = 0

        # Produz o steer a partir do modo ATIVO (pós-debounce), não do
        # raw_mode deste frame isolado — é isso que impede o salto brusco.
        if self._active_mode == "LANE":
            if lane_center is not None:
                offset_px = lane_center - image_center
                raw_steer = float(np.clip((offset_px / (w / 2.0)) * 1.2, -1.0, 1.0))
                # EMA: suaviza com o steer anterior em vez de substituir direto.
                with self._lock:
                    prev_steer = self.steer
                steer = STEER_SMOOTHING * prev_steer + (1.0 - STEER_SMOOTHING) * raw_steer
                throttle = lane_throttle
            else:
                with self._lock:
                    steer = self.steer
                throttle = DRIVE_THROTTLE_DEFAULT
        else:  # NO_LANE
            with self._waypoint_lock:
                wp_steer = self._waypoint_steer
            if wp_steer is not None:
                steer = wp_steer
            else:
                with self._lock:
                    steer = self.steer
            throttle = NO_LANE_THROTTLE

        if self.show_debug and lane_center is not None:
            dbg = img.copy()
            cv2.polylines(dbg, roi, True, (0, 255, 255), 1)
            cv2.circle(dbg, (int(lane_center), lookahead_y), 6, (0, 0, 255), -1)
            if lines is not None:
                for x1, y1, x2, y2 in lines.reshape(-1, 4):
                    cv2.line(dbg, (x1, y1), (x2, y2), (0, 255, 0), 2)
            thumb_w, thumb_h = w // 4, h // 4
            mask_thumb = cv2.resize(color_mask, (thumb_w, thumb_h))
            mask_thumb_bgr = cv2.cvtColor(mask_thumb, cv2.COLOR_GRAY2BGR)
            dbg[0:thumb_h, 0:thumb_w] = mask_thumb_bgr
            try:
                self._debug_queue.put_nowait(dbg)
            except queue.Full:
                try:
                    self._debug_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._debug_queue.put_nowait(dbg)
                except queue.Full:
                    pass

        if self._last_mode != self._active_mode:
            if self._active_mode == "LANE":
                print(f"[camera_autopilot] {time.strftime('%H:%M:%S')} "
                      f"Faixa visível — lane detection no controle (steer={steer:.2f})")
            else:
                with self._waypoint_lock:
                    wp_available = self._waypoint_steer is not None
                if wp_available:
                    print(f"[camera_autopilot] {time.strftime('%H:%M:%S')} "
                          f"Sem faixa visível — usando waypoint do mapa como direção "
                          f"temporária (steer={steer:.2f})")
                else:
                    print(f"[camera_autopilot] {time.strftime('%H:%M:%S')} "
                          f"Sem faixa visível e sem waypoint disponível — mantendo "
                          f"último steer conhecido ({steer:.2f})")
            self._last_mode = self._active_mode

        return steer, throttle, self._active_mode == "LANE"

    def _debug_display_loop(self):
        """Thread própria só pra escrever o frame de debug em disco (sem
        cv2.imshow — travava o processo inteiro nesse ambiente Qt/Wayland).
        Escreve em arquivo temporário e troca de nome (os.replace) pra não
        deixar um visualizador externo abrir o arquivo pela metade. O
        ".tmp" precisa vir ANTES da extensão real (".tmp.jpg", não
        ".jpg.tmp") — o cv2.imwrite decide o formato pela extensão do nome.
        """
        root, ext = os.path.splitext(self.debug_image_path)
        tmp_path = f"{root}.tmp{ext}"
        while self.running:
            try:
                dbg = self._debug_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                cv2.imwrite(tmp_path, dbg)
                os.replace(tmp_path, self.debug_image_path)
            except Exception as e:
                print(f"[camera_autopilot] Falha ao escrever frame de debug "
                      f"(ignorando, autopilot continua normal): {e}")

    def _decode_loop(self):
        """Thread servidora: aceita a conexão do receiver.py e decodifica o stream continuamente."""
        if os.path.exists(self.socket_path):
            os.remove(self.socket_path)

        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(self.socket_path)
        srv.listen(1)
        print(f"[camera_autopilot] Esperando receiver.py conectar em {self.socket_path} ...")

        while self.running:
            conn, _ = srv.accept()
            print("[camera_autopilot] receiver.py conectado — decodificando stream.")
            try:
                self._decode_stream(conn)
            except Exception as e:
                # Qualquer exceção aqui só significa "essa conexão acabou"
                # (EOF, dados inválidos, conexão caída) — volta a aceitar.
                print(f"[camera_autopilot] Stream encerrado/interrompido: {e}")
                traceback.print_exc()
            finally:
                conn.close()
            print("[camera_autopilot] Aguardando nova conexão do receiver.py...")

    @staticmethod
    def _recv_exact(conn: socket.socket, n: int) -> bytes | None:
        """Lê exatamente n bytes do socket, ou None se a conexão fechou no meio."""
        buf = bytearray()
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    @classmethod
    def _recv_framed_packet(cls, conn: socket.socket) -> bytes | None:
        """Lê um pacote AVTP bruto framed (4B tamanho + payload) do socket —
        contraparte de _cam_feed_loop em receiver.py."""
        header = cls._recv_exact(conn, 4)
        if header is None:
            return None
        (length,) = struct.unpack("!I", header)
        if length == 0 or length > MAX_RAW_PACKET_SIZE:
            return None
        return cls._recv_exact(conn, length)

    def _decode_stream(self, conn: socket.socket):
        """receiver.py encaminha pacotes AVTP brutos framed (corrompidos ou
        não) — o desempacotamento MPEG-TS (parse_mpegts_stream_packet) é
        feito aqui. PyAV só lê um stream MPEG-TS contínuo, então uma thread
        separada lê os pacotes do socket, desempacota cada um e escreve o
        ts_data extraído num pipe; o PyAV lê do outro lado."""
        read_fd, write_fd = os.pipe()
        pipe_reader = os.fdopen(read_fd, "rb")
        pipe_writer = os.fdopen(write_fd, "wb")
        stop_unpack = threading.Event()

        def _unpack_loop():
            try:
                while not stop_unpack.is_set():
                    raw_pkt = self._recv_framed_packet(conn)
                    if raw_pkt is None:
                        break
                    ts_data = parse_mpegts_stream_packet(raw_pkt)
                    if not ts_data:
                        continue  # sem sync byte (ex: corrompido) — descarta e segue
                    try:
                        pipe_writer.write(ts_data)
                        pipe_writer.flush()
                    except (BrokenPipeError, OSError):
                        break
            finally:
                try:
                    pipe_writer.close()
                except OSError:
                    pass

        unpack_thread = threading.Thread(target=_unpack_loop, daemon=True)
        unpack_thread.start()

        try:
            # probesize/analyzeduration maiores: evita o PyAV quebrar
            # sondando framerate com poucos bytes num stream ao vivo.
            container = av.open(
                pipe_reader, format="mpegts", mode="r",
                options={"probesize": "5000000", "analyzeduration": "5000000"},
            )
            try:
                for frame in container.decode(video=0):
                    if not self.running:
                        break
                    img = frame.to_ndarray(format="bgr24")
                    steer, throttle, visible = self._lane_detect(img)
                    with self._lock:
                        self.steer = steer
                        self.throttle = throttle
                        self.lanes_visible = visible
            finally:
                container.close()
        finally:
            stop_unpack.set()
            try:
                pipe_reader.close()
            except OSError:
                pass
            unpack_thread.join(timeout=1)

    # ------------------------------------------------------------------
    # Fallback de waypoint (CARLA RPC, só consulta — ver parse_args)
    # ------------------------------------------------------------------

    def _ensure_carla_connection(self) -> bool:
        """Garante client/world/veículo ego prontos pra consulta. Retorna
        True se pode consultar waypoint agora. Barato quando já está tudo
        conectado (só checa os campos); só tenta de fato conectar/localizar
        o ator quando algo ainda falta."""
        if self._carla_world is not None and self._ego_vehicle is not None:
            # Veículo pode ter sido destruído (BACKSPACE em
            # keyboard_control.py) sem avisar este processo — is_alive
            # detecta isso sem precisar de uma chamada RPC cara.
            if self._ego_vehicle.is_alive:
                return True
            self._ego_vehicle = None

        try:
            if self._carla_client is None:
                self._carla_client = carla.Client(self.carla_host, self.carla_port)
                self._carla_client.set_timeout(2.0)
            if self._carla_world is None:
                self._carla_world = self._carla_client.get_world()
                self._carla_map = self._carla_world.get_map()
            if self._ego_vehicle is None:
                for actor in self._carla_world.get_actors().filter("vehicle.*"):
                    if actor.attributes.get("role_name", "") == self.carla_rolename:
                        self._ego_vehicle = actor
                        break
            connected = self._carla_world is not None and self._ego_vehicle is not None
        except Exception:
            # Servidor caiu/reiniciou ou conexão expirou — descarta tudo pra
            # forçar reconexão completa na próxima tentativa.
            self._carla_client = None
            self._carla_world = None
            self._carla_map = None
            self._ego_vehicle = None
            connected = False

        # Loga só na transição conectado <-> desconectado, não a cada chamada.
        if connected != self._wp_connected_logged:
            if connected:
                print(f"[camera_autopilot] {time.strftime('%H:%M:%S')} "
                      f"Conectado ao CARLA ({self.carla_host}:{self.carla_port}) — "
                      f"veículo '{self.carla_rolename}' localizado, fallback de "
                      f"waypoint disponível.")
            else:
                print(f"[camera_autopilot] {time.strftime('%H:%M:%S')} "
                      f"Sem conexão/veículo CARLA disponível — fallback de waypoint "
                      f"desativado por ora.")
            self._wp_connected_logged = connected
        return connected

    def _update_waypoint_steer(self):
        """Calcula o steer rumo ao próximo waypoint do mapa e guarda em
        self._waypoint_steer, pra _lane_detect consumir sem faixa visível."""
        if not self._ensure_carla_connection():
            with self._waypoint_lock:
                self._waypoint_steer = None
            return
        try:
            transform = self._ego_vehicle.get_transform()
            location = transform.location
            current_wp = self._carla_map.get_waypoint(location, project_to_road=True)
            next_wps = current_wp.next(WAYPOINT_LOOKAHEAD_METERS)
            if not next_wps:
                with self._waypoint_lock:
                    self._waypoint_steer = None
                return
            target = next_wps[0].transform.location
            dx = target.x - location.x
            dy = target.y - location.y
            target_yaw = math.degrees(math.atan2(dy, dx))
            # Diferença entre o rumo atual do carro e o rumo até o próximo
            # waypoint, normalizada pra [-180, 180] (sem isso, uma diferença
            # tipo "350°" seria tratada como quase uma volta completa em vez
            # de só 10° pro outro lado).
            yaw_diff = (target_yaw - transform.rotation.yaw + 180.0) % 360.0 - 180.0
            raw_wp_steer = max(-1.0, min(1.0, yaw_diff / WAYPOINT_STEER_DEG_RANGE))
            # Mesma suavização EMA do steer de lane detection.
            with self._waypoint_lock:
                prev_wp_steer = self._waypoint_steer
                if prev_wp_steer is None:
                    self._waypoint_steer = raw_wp_steer
                else:
                    self._waypoint_steer = (STEER_SMOOTHING * prev_wp_steer
                                             + (1.0 - STEER_SMOOTHING) * raw_wp_steer)
        except Exception:
            with self._waypoint_lock:
                self._waypoint_steer = None
            self._carla_world = None
            self._ego_vehicle = None

    def _check_stuck(self):
        """Detecta o carro parado (ex: raspando meio-fio em baixa
        intensidade, sem disparar o CollisionSensor) via posição real (RPC),
        já que CAN não carrega velocidade do veículo. Se throttle está
        sendo comandado mas o carro não andou STUCK_MIN_DISTANCE em
        STUCK_CHECK_WINDOW segundos, força RECOVER.

        Só roda com o autopilot LIGADO ('K'): antes disso self.throttle já
        vem >0 da thread de decode (mantém a visão "aquecida"), mas nada
        está de fato comandando o carro — sem esse guard, _check_stuck
        achava "parado" em qualquer conexão e deixava self.state preso em
        RECOVER, que _build_control então aplicava assim que o 'K' fosse
        apertado (carro já começando de ré)."""
        if not self.can_net.cam_autopilot_enabled:
            self._stuck_ref_location = None
            self._stuck_ref_time = None
            return
        if self.state != "DRIVE":
            self._stuck_ref_location = None
            self._stuck_ref_time = None
            return
        if self._ego_vehicle is None or not self._ego_vehicle.is_alive:
            return
        with self._lock:
            commanded_throttle = self.throttle
        if commanded_throttle <= 0.0:
            return

        # Usa o relógio de simulação do CARLA (elapsed_seconds, avança
        # fixed_delta_seconds=0.05 por world.tick()), não time.time(): em
        # synchronous_mode o tempo simulado só avança quando o client
        # (CARLA_client_module.py) chama tick(), então se o FPS real do
        # client cair (CPU disputada), 3s de relógio de parede podem
        # corresponder a muito menos tempo simulado de verdade — o carro
        # "parece" parado só porque a física também andou em câmera lenta,
        # e o STUCK_CHECK_WINDOW estourava rápido demais pra qualquer
        # throttle baixo conseguir mover STUCK_MIN_DISTANCE. Medindo o
        # próprio tempo simulado, a janela só fecha quando o veículo de
        # fato teve STUCK_CHECK_WINDOW segundos de física pra se mover.
        try:
            loc = self._ego_vehicle.get_location()
            sim_now = self._carla_world.get_snapshot().timestamp.elapsed_seconds
        except Exception:
            return

        if self._stuck_ref_location is None:
            self._stuck_ref_location = loc
            self._stuck_ref_time = sim_now
            return
        if sim_now - self._stuck_ref_time < STUCK_CHECK_WINDOW:
            return

        dist = math.hypot(loc.x - self._stuck_ref_location.x, loc.y - self._stuck_ref_location.y)
        self._stuck_ref_location = loc
        self._stuck_ref_time = sim_now
        if dist < STUCK_MIN_DISTANCE:
            print(f"[camera_autopilot] {time.strftime('%H:%M:%S')} "
                  f"Carro parado apesar do throttle (andou {dist:.2f}m em "
                  f"{STUCK_CHECK_WINDOW:.0f}s simulados) — forçando RECOVER "
                  f"mesmo sem colisão detectada pelo sensor")
            now = time.time()
            self.state = "RECOVER"
            self.state_entered_at = now
            self._recover_started_at = now
            self._recover_from_stuck = True

    # ------------------------------------------------------------------
    # Máquina de estados de colisão + laço de controle
    # ------------------------------------------------------------------

    def _update_collision_state(self):
        """Detecta colisão nova (publicada por sensors/collision.py via CAN) e
        decide a transição de estado. recv_msg() também atualiza
        self.can_net.cam_autopilot_enabled, lido no laço de controle.

        recv_msg() e o consumo do contador de colisão rodam sempre (pra não
        acumular um "new_collisions" antigo que estoura assim que o
        autopilot for ligado), mas as transições de estado (RECOVER/
        REORIENT/DRIVE) só se aplicam com o autopilot LIGADO — mesma razão
        do guard em _check_stuck: sem isso, self.state ficava mudando por
        conta própria antes do 'K', e _build_control herdava esse estado
        velho no primeiro ciclo depois de ligar."""
        self.can_net.recv_msg()
        now = time.time()

        new_collisions = self.can_net.collision_event_count - self._last_collision_count
        self._last_collision_count = self.can_net.collision_event_count

        if not self.can_net.cam_autopilot_enabled:
            return

        if new_collisions > 0:
            intensity = self.can_net.last_collision_intensity
            if self.state == "DRIVE":
                print(f"[camera_autopilot] Colisão detectada "
                      f"(intensidade={intensity:.1f}) — iniciando ré")
                self.state = "RECOVER"
                self.state_entered_at = now
                self._recover_started_at = now
                self._recover_from_stuck = False
            elif self.state in ("RECOVER", "REORIENT") and intensity >= MIN_PERSIST_INTENSITY:
                # Quanto tempo de ré já foi gasto NESTE episódio de colisão
                # (desde a entrada original em RECOVER, não desde a última
                # extensão) — calculado ANTES de decidir se estende de novo.
                total_so_far = (now - self._recover_started_at) if self._recover_started_at else 0.0
                if total_so_far < MAX_RECOVER_TOTAL:
                    # Ainda está colidindo de verdade (acima do ruído
                    # mínimo). Em vez de deixar o timer da fase atual
                    # estourar e seguir pra DRIVE/REORIENT igual, volta pra
                    # RECOVER e reinicia o timer DA FASE — estende a ré até
                    # realmente parar de colidir (ou até bater no teto).
                    print(f"[camera_autopilot] Colisão persistindo em {self.state} "
                          f"(intensidade={intensity:.1f}) — estendendo ré")
                    self.state = "RECOVER"
                    self.state_entered_at = now
                    self._recover_from_stuck = False
                else:
                    # Já no teto de MAX_RECOVER_TOTAL: NÃO volta mais pra
                    # RECOVER, mesmo com colisão persistindo. Sem este
                    # "else", o ciclo abaixo (que força REORIENT ao bater
                    # no teto) e este daqui (que volta pra RECOVER ao ver
                    # colisão persistindo) ficam se cutucando — força
                    # REORIENT, no ciclo seguinte ainda está colidindo,
                    # volta pra RECOVER, que de novo já passou do teto,
                    # força REORIENT nos próximos 100ms de novo, pra
                    # sempre, sem nunca completar os REORIENT_DURATION
                    # segundos necessários pra voltar a DRIVE (foi
                    # exatamente o que aconteceu nos logs — "forçando
                    # REORIENT" alternando com "estendendo ré" sem fim).
                    # Aqui simplesmente ignora e deixa o REORIENT/DRIVE
                    # completarem — desiste de tentar sair por ré desse
                    # obstáculo específico.
                    print(f"[camera_autopilot] Colisão ainda persistindo em {self.state} "
                          f"(intensidade={intensity:.1f}), mas teto de {MAX_RECOVER_TOTAL}s "
                          f"já atingido — ignorando, deixando completar")

        elapsed = now - self.state_entered_at
        total_recover_elapsed = (
            now - self._recover_started_at if self._recover_started_at is not None else 0.0
        )
        if self.state == "RECOVER" and (
            elapsed > RECOVER_DURATION or total_recover_elapsed > MAX_RECOVER_TOTAL
        ):
            if total_recover_elapsed > MAX_RECOVER_TOTAL:
                print(f"[camera_autopilot] Ré total passou de {MAX_RECOVER_TOTAL}s "
                      f"sem sair da colisão — forçando REORIENT mesmo assim "
                      f"(provável preso num canto/parede)")
            self.state = "REORIENT"
            self.state_entered_at = now
        elif self.state == "REORIENT" and elapsed > REORIENT_DURATION:
            self.state = "DRIVE"
            self.state_entered_at = now
            self._recover_started_at = None
            # A thread de decode continua rodando (e atualizando
            # self.steer/self.throttle) o tempo todo durante RECOVER/
            # REORIENT — só que com a imagem distorcida/perto da parede
            # durante a colisão, então o último valor calculado pode ser
            # um esterço extremo. Sem resetar aqui, o DRIVE herdava esse
            # valor velho de uma vez só ao retomar, causando um esterço
            # brusco logo na saída da recuperação. Reseta pra 0 só neste
            # instante da transição — o próximo frame decodificado (a
            # ~10Hz) já substitui por um valor fresco e são, retomando o
            # lane-following normalmente.
            with self._lock:
                self.steer = 0.0
                self.throttle = DRIVE_THROTTLE_DEFAULT
            print("[camera_autopilot] Recuperação concluída — retomando lane detection")

    def _build_control(self) -> carla.VehicleControl:
        ctrl = carla.VehicleControl()
        if self.state == "RECOVER":
            ctrl.reverse = True
            ctrl.throttle = RECOVER_THROTTLE
            ctrl.brake = 0.0
            # Colisão real: esterço fixo pra se afastar do que bateu. Preso
            # sem colisão (_check_stuck): sem saber de que lado está o
            # obstáculo, dá ré reto — esterçar sempre pro mesmo lado
            # arrisca reencostar no mesmo obstáculo.
            ctrl.steer = 0.0 if self._recover_from_stuck else 0.5
        elif self.state == "REORIENT":
            ctrl.reverse = False
            ctrl.throttle = 0.0
            ctrl.brake = 1.0
            ctrl.steer = 0.0
        else:  # DRIVE
            with self._lock:
                ctrl.steer = max(-1.0, min(1.0, self.steer))
                ctrl.throttle = self.throttle
            ctrl.reverse = False
            ctrl.brake = 0.0

        # self.can_net.recvd_controls (do lado de quem RECEBE, em
        # CARLA_client_module.py) é um único carla.VehicleControl()
        # persistente — campos que a gente nunca manda aqui ficam travados
        # no último valor publicado por QUALQUER sender anterior (ex:
        # vehicle_controls_module.py, antes da tecla 'K' ser apertada). Se
        # o freio de mão ficou travado em True (SPACE segurado antes), ou
        # manual_gear_shift ficou True com uma marcha errada/neutra (tecla
        # M antes), o carro não anda nunca, mesmo com throttle/steer
        # corretos chegando. Fixamos esses campos aqui também, pra nunca
        # depender de estado residual de outro processo.
        ctrl.hand_brake = False
        ctrl.manual_gear_shift = False
        ctrl.gear = 1
        return ctrl

    def _control_loop(self):
        period = 1.0 / CONTROL_HZ
        while self.running:
            t0 = time.time()
            self._update_collision_state()
            # _update_collision_state fica de fora da "divisão" (só lê CAN,
            # local e barato) — só as duas chamadas que batem no servidor
            # CARLA via RPC são reduzidas em frequência.
            self._control_tick += 1
            if self._control_tick % RPC_POLL_DIVIDER == 0:
                self._update_waypoint_steer()
                self._check_stuck()

            enabled = self.can_net.cam_autopilot_enabled
            if enabled:
                ctrl = self._build_control()
                self.can_net.send_throttle_msg(ctrl)
                self.can_net.send_steer_msg(ctrl)
                self.can_net.send_brake_msg(ctrl)
                self.can_net.send_reverse_msg(ctrl)
                self.can_net.send_hand_brake_msg(ctrl)
                self.can_net.send_manual_transmission_msg(ctrl)
                self.can_net.send_gear_msg(ctrl)
                self._was_enabled = True
            elif self._was_enabled:
                # Transição de ligado -> desligado: manda um controle seguro
                # uma vez, pra não deixar o throttle "travado" no último
                # valor comandado quando outra fonte assumir o veículo.
                safe = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
                self.can_net.send_throttle_msg(safe)
                self.can_net.send_brake_msg(safe)
                self._was_enabled = False
                self.state = "DRIVE"

            elapsed = time.time() - t0
            time.sleep(max(0.0, period - elapsed))

    def start(self):
        decode_thread = threading.Thread(target=self._decode_loop, daemon=True)
        decode_thread.start()
        if self.show_debug:
            display_thread = threading.Thread(target=self._debug_display_loop, daemon=True)
            display_thread.start()
        print("[camera_autopilot] Pronto. Ative com a tecla 'K' na janela do CARLA_client_module.py.")
        try:
            self._control_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False


def main():
    args = parse_args()
    autopilot = CameraAutopilot(args)
    autopilot.start()


if __name__ == "__main__":
    main()