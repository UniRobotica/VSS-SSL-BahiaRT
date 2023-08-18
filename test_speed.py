import time
import argparse

# Definindo ambiente -----------------------------------------------------------

parser = argparse.ArgumentParser(description='BahiaRT_V3S')
parser.add_argument('--env', default='simulation') # real or simulation
args = parser.parse_args()

# Criando comunicação ----------------------------------------------------------

# Para o teste real
if args.env == 'real':
    
    from vision import visionReal
    from comm import serial

    # Criando comunicação
    vision = visionReal.VisionReal()
    serial_comm = serial.SerialComm()
    serial_comm.start() # Inicializando comunicação serial

# Para o teste simulado
else:
    
    from vision import visionSim

    # Criando comunicação com a visão
    vision = visionSim.VisionSim()

vision.start() # Inicializando comunicação com a visão

# Criando entidades ---------------------------------------------------------

from entities.Ball import Ball

ball = Ball()
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    
    while True:

        time.sleep(0.003) # Necessário para o recebimento correto da informação da visão
        
        if vision.frame:
                
            print('Vision data received')
            
            while True:
                
                time.sleep(0.003) # Necessário para o recebimento correto da informação da visão
                
                ball.update(vision.frame)
                print('vx:', ball.vx)
                print('vy:', ball.vy)
                print(' ')

     
        else:
            print('Waiting for vision data...')
            time.sleep(3)