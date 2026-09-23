'''
Mock de IA: código que simula uma IA ao fazer uma decisão a partir de um cálculo simples feito a partir 
Código gerado majoritariamente pelo Gemini 3.6 Raciocínio, sob revisão e adições da pesquisadora.
'''

import numpy as np 
import time
from random import randint


def prosseguir_ou_frear(imagem_rgb):

    # verificando se a imagem recebida está no formato correto (tridimensional) 
    if imagem_rgb.shape != 3:
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

    return throttle, brake

# sugestão para versões futuras: considerar utilizar o comando sensor.camera.depth do próprio CARLA 





    