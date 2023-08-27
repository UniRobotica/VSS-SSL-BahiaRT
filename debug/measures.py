import math
from random import randint
from typing import Tuple
import numpy as np

# Pixel converter
arena_w = 150 # cm
arena_h = 130 # cm
default_convertion_factor = 0.045
img_w = 675 # px
img_h = 585 # px
step = 3 # cm

# Gets the height/width or width/height proportion
height_to_width = arena_w / arena_h
width_to_height = arena_h / arena_w

if img_w:
    convertion_factor = img_w / arena_w
elif img_h:
    convertion_factor = img_h / arena_h
else:
    convertion_factor = default_convertion_factor

def randomPosition():
    
    return np.array((randint(10, arena_w - 10), randint(10, arena_h - 10))) / 100

def centerPosition():
    
    return np.array([arena_w/2, arena_h/2]) / 100

def getArenaSize() -> Tuple[int, int]:

    global img_w
    global img_h
    global arena_w
    global arena_h

    if not img_w and not img_h:
        img_w = 150
        img_h = 130

    elif not img_w:
        img_w = round(height_to_width * img_h)

    elif not img_h:
        img_h = round(width_to_height * img_w)

    return img_w, img_h

def convertToPixel(m: float) -> int:

    return round(m * convertion_factor)

def convertToCm(px: float) -> int:
    
    return round(px / convertion_factor)