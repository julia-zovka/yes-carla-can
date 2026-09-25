# ATENCAO: essa é a versao inicial do camera.py, sem as alteracoes  relativas à rede AVTP. Após o teste inicial do ai_mock, é necessário trocar pela versao atualizada do camera.py

import os
import weakref

import numpy as np
import pygame
import carla
import time
from random import randint 

def prosseguir_ou_frear(imagem_rgb):

    # verificando se a imagem recebida está no formato correto (tridimensional) 
    if imagem_rgb.shape[2] != 3:
        print(f"Formato inválido. Imagem com {imagem_rgb.shape} dimensões.\n")
        return 
    
    altura, largura, _ = imagem_rgb.shape

    # definindo um polígono como nossa região de interesse (quadrado logo à frente do carro, no centro inferior)
    roi = imagem_rgb[int(altura*0.7):int(altura*0.9), int(altura*0.4):int(largura*0.6)]
    
    # convertendo para esscala de cinza de forma simples
    gray_roi = np.mean(roi, axis=2)

    # contando pixels que estão na faixa de cor do asfalto (dentro da escala de cinza)
    pixels_pista = np.sum((gray_roi > 60) & (gray_roi < 120)) # valores estimados
    total_pixels = gray_roi.size 
    proporcao_pista = pixels_pista / total_pixels

    # gerando ruído, para simular oscilações no valor final gerado por um modelo mais robusto
    random1 = randint(10, 50)
    random2 = randint(10, 50)
    ruido1 = random1/100
    ruido2 = random2/100

    # se mais de 40% da área for de pixels na escala do asfalto, consideramos que o caminho está livre, logo, o carro será acelerado
    if proporcao_pista > 0.4:

        throttle = 0.3 - ruido1
        brake = 0.0 + ruido2


    # do contrário, o carro deve ser freado
    else: 

        throttle = 0.0 + ruido1
        brake = 1.0 - ruido2

    # simulando uma "demora" de inferência, que deve acontecer quando colocarmos um modelo de verdade para funcionar
    time.sleep(0.03)

    print(f"Array inicial: {imagem_rgb}; throttle: {throttle}, brake: {brake}.\n")

    return throttle, brake

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

    def __init__(self, parent_actor, gamma_correction=2.2):
        self.sensor = None
        self.surface = None          # pygame.Surface, updated each frame
        self.array = None            # numpy (H, W, 3) uint8 RGB, updated each frame
        self.recording = False       # set True to auto-save every frame to disk

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

        # Drop alpha channel and convert BGRA → RGB for conventional use.
        array = array[:, :, :3][:, :, ::-1]

        # Store numpy array (RGB, uint8) — use this for data export.
        self.array = array

        prosseguir_ou_frear(array)


        # Build pygame surface (expects (W, H) axis order).
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        if self.recording:
            # CARLA saves the raw BGRA image; the frame number is used as
            # the file name.  Files land in ``_out/camera/<frame>.png``.
            os.makedirs('_out/camera', exist_ok=True)
            image.save_to_disk('_out/camera/%08d' % image.frame)
