# colocando o fluxo completo num só arquivo, mas o objetivo futuro é modularizá-lo

import numpy as np 
import time
from random import randint
import sys 
from pathlib import Path
import carla
from queue import LifoQueue
from can_network.network import CAN_Network

# encontrando o caminho dos imports em relação ao diretório do repositório 
curr_folder_abs_path = Path(__file__).resolve() # encontrando o caminho absoluto da pasta atual

# enquanto não chegamos na pasta do diretório (e enquanto o pai da pasta não é ela própria, indicando que não há mais níveis a subir), sobrescrevemos a pasta atual pelo seu pai 
while curr_folder_abs_path.name != "yes-carla-can" and curr_folder_abs_path.parent != curr_folder_abs_path:
    curr_folder_abs_path =  curr_folder_abs_path.parent

# se saímos do while, é porque encontramos o caminho correto 
sys.path.insert(0, str(curr_folder_abs_path)) # colocando encontrado como o primeiro na fila de busca por módulos e arquivos

from sensors.camera import RGBCameraSensor
from gui.world import World

def throttle_or_brake(imagem_rgb):

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


    '''
    # se mais de 40% da área for de pixels na escala do asfalto, consideramos que o caminho está livre, logo, o carro será acelerado
    if proporcao_pista > 0.4:

        throttle = min(0.3 - ruido1, 1.0)
        #brake = min(0.0 + ruido2, 1.0)


    # do contrário, o carro deve ser freado
    else: 

        throttle = min(0.0 + ruido1, 1)
        brake = min(1.0 - ruido2, 1) 

    # simulando uma "demora" de inferência, que deve acontecer quando colocarmos um modelo de verdade para funcionar
    time.sleep(0.03)

    '''

    throttle = 0.6
    brake = 0.0

    print(f"Array inicial: {imagem_rgb}; throttle: {throttle}, brake: {brake}.\n")

    return throttle, brake

class ai_control(object):

    def __init__(self, throttle, brake):

        self.throttle = throttle
        self.brake = brake

