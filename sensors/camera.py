import os
import sys
import time
import weakref

import numpy as np
import pygame
import carla


PATH_AVTP = "/home/ju/virtual-avtp-network"  # Coloque o caminho absoluto da sua pasta do AVTP aqui
if PATH_AVTP not in sys.path:
    sys.path.append(PATH_AVTP)


import cv2
import avtp as avtp_lib
from scapy.all import sendp, get_if_hwaddr, conf 


class RGBCameraSensor(object):
    """
    Dedicated front-facing RGB camera sensor.

    Stores the latest frame both as a pygame Surface (for on-screen display)
    and as a raw numpy array (for data export / ML pipelines).

    ── How to access the camera data ──────────────────────────────────────────

    From anywhere that holds a reference to this sensor object:

        # Latest frame as a numpy uint8 array shaped (H, W, 3) in RGB order
        frame_rgb = world.rgb_camera_sensor.array

        # Save the current frame to a PNG file (requires Pillow):
        from PIL import Image
        img = Image.fromarray(frame_rgb)
        img.save("frame.png")

        # Or use OpenCV:
        import cv2
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite("frame.png", frame_bgr)

    To export every frame automatically, set ``recording = True`` on the
    sensor instance.  Frames will be saved under the ``_out/camera/``
    directory as ``<frame_number>.png`` via CARLA's built-in save helper:

        world.rgb_camera_sensor.recording = True   # start
        world.rgb_camera_sensor.recording = False  # stop

    ───────────────────────────────────────────────────────────────────────────
    """

    # Resolution of the camera (pixels).  Must match or be smaller than the
    # pygame display so the PiP overlay fits on screen.
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 360

    def __init__(self, parent_actor, gamma_correction=2.2, stream_id="0xAABBCCDDEEFF0001", interface="veth-s"):
        self.sensor = None
        self.surface = None          # pygame.Surface, updated each frame
        self.array = None            # numpy (H, W, 3) uint8 RGB, updated each frame
        self.recording = False       # set True to auto-save every frame to disk


        # ── Configurações de Rede AVTP ───────────────────────────────────────
        self.interface = interface
        self.stream_id = int(stream_id, 16)


        try:
            self.src_mac = get_if_hwaddr(self.interface)
        except Exception as exc:
            print(f"[!] Erro ao ler MAC da interface '{self.interface}': {exc}")
            sys.exit(1)

        conf.iface = self.interface    
        
        # Contadores do AVTP
        self.seq_counter = 0     # AVTP sequence_num (wraps at 255)
        self.frames_sent = 0
        self.pkts_sent = 0
        self.bytes_sent = 0
        self.start_time = time.time()
        # ─────────────────────────────────────────────────────────────────────



        self._parent = parent_actor

        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(self.IMAGE_WIDTH))
        bp.set_attribute('image_size_y', str(self.IMAGE_HEIGHT))
        bp.set_attribute('fov', '90')
        if bp.has_attribute('gamma'):
            bp.set_attribute('gamma', str(gamma_correction))

        # Mount on the front hood of the vehicle, slightly elevated.
        spawn_transform = carla.Transform(
            carla.Location(x=bound_x + 0.3, z=bound_z + 0.1),
            carla.Rotation(pitch=0.0),
        )

        self.sensor = world.spawn_actor(
            bp,
            spawn_transform,
            attach_to=self._parent,
            attachment_type=carla.AttachmentType.Rigid,
        )

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda image: RGBCameraSensor._on_image(weak_self, image)
        )

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def render(self, display, pos=(0, 0)):
        """Blit the latest camera frame onto *display* at *pos* (top-left)."""
        if self.surface is not None:
            display.blit(self.surface, pos)

    # ------------------------------------------------------------------
    # Internal callback
    # ------------------------------------------------------------------

    @staticmethod
    def _on_image(weak_self, image):
        self = weak_self()
        if not self:
            return

        # raw_data is a flat BGRA byte buffer; reshape to (H, W, 4).
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))


        # Mantém o BGRA apenas para o OpenCV codificar para JPEG
        bgra_array = array

       # Prepara a exibição no PyGame
        rgb_array = array[:, :, :3][:, :, ::-1]
        self.array = rgb_array

        if array is not None and len(array.shape) == 3 and array.shape[2] >= 3:
            # Garante que pegamos apenas os canais RGB (primeiros 3 canais)
            rgb_array = array[:, :, :3]
            self.surface = pygame.surfarray.make_surface(rgb_array.swapaxes(0, 1))
                
        # ── Codificação e Envio AVTP (Substitui a leitura de arquivo do sender antigo)
        # Transmissão em Tempo Real via AVTP
        # Converte o frame em JPEG compactado diretamente em RAM (Qualidade 80%)
        success, buffer = cv2.imencode(".jpg", bgra_array, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not success:
            return
        
        image_bytes = buffer.tobytes()
        frame_num = self.frames_sent & 0xFFFF

        # Fragmenta em pacotes AVTP
        packets = avtp_lib.fragment_image(
            image_bytes=image_bytes,
            stream_id=self.stream_id,
            frame_num=frame_num,
            seq_counter=self.seq_counter,
            src_mac=self.src_mac,
        )

        # Envia os fragmentos pela interface de rede especificada
        sendp(packets, iface=self.interface, verbose=0)

        # Atualiza os contadores internos
        self.seq_counter = (self.seq_counter + len(packets)) & 0xFF
        self.frames_sent += 1
        self.bytes_sent += len(image_bytes)
        self.pkts_sent += len(packets)

        elapsed = time.time() - self.start_time
        print(
            f"  [TX] Frame {self.frames_sent:>5}"
            f"  {len(image_bytes):>7} B"
            f"  {len(packets):>3} pkts"
            f"  seq {frame_num:>5}"
            f"  elapsed {elapsed:>7.1f}s"
        )