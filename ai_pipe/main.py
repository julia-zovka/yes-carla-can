import sys 
from pathlib import Path

# encontrando o caminho dos imports em relação ao diretório do repositório 
curr_folder_abs_path = Path(__file__).resolve() # encontrando o caminho absoluto da pasta atual

# enquanto não chegamos na pasta do diretório (e enquanto o pai da pasta não é ela própria, indicando que não há mais níveis a subir), sobrescrevemos a pasta atual pelo seu pai 
while curr_folder_abs_path.name != "yes-carla-can" and curr_folder_abs_path.parent != curr_folder_abs_path:
    curr_folder_abs_path =  curr_folder_abs_path.parent

# se saímos do while, é porque encontramos o caminho correto 
sys.path.insert(0, str(curr_folder_abs_path)) # colocando encontrado como o primeiro na fila de busca por módulos e arquivos

# instanciando a câmera (versão sem AVTP)
from sensors import camera as camera

camera = camera.RGBCameraSensor(recording = False);
print(f"{camera.RGBCameraSensor.recording}")