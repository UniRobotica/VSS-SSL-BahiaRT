from entities.Ball import Ball
from entities.Robot import Robot

import time
import argparse

parser = argparse.ArgumentParser(description='BahiaRT_V3S')
parser.add_argument('--env', default='simulation') # real or simulation
args = parser.parse_args()

# Para o teste real
if args.env == 'real':
    
    from vision import visionReal

    # Criando comunicação
    vision = visionReal.VisionReal()

# Para o teste simulado
else:
    
    from vision import visionSim

    # Criando comunicação
    vision = visionSim.VisionSim()

# Inicializando comunicação
vision.start()

if __name__ == '__main__':
    
    while True:

        time.sleep(0.003) # Necessário para o recebimento correto da informação da visão
        
        if vision.frame:
                
            print('Vision data received')
            
            while True:
                
                time.sleep(0.003) # Necessário para o recebimento correto da informação da visão
                
                vision.printInfo()
     
        else:
            print('Waiting for vision data...')
            vision.printInfo()
            time.sleep(3)