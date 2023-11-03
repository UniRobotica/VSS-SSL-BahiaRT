from copy import copy
import cv2
import math
from utils import util
import numpy as np
from debug import measures, draw
from algorithms import univector_field

angle = math.radians(0)

def debug(name: str, univector_field: univector_field.BaseField, obstacles: list[list[int]]) -> None:

    w, h = measures.arena_w, measures.arena_h
    img_w, img_h = measures.getArenaSize()
    step = measures.step
    
    field = np.full((img_h, img_w, 3), 255, dtype=np.uint8)
    vectors = getVectors(w, h, step, univector_field, obstacles)
    vectorField = draw.drawVectorField(copy(field), vectors, w, h, step, univector_field.home_point * 100, obstacles)

    cv2.imshow('{}'.format(name), vectorField)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def getVectors(w: int, h: int, step: int, get_vec: univector_field.BaseField, obstacles: list[list[int]]=None) -> list[list[float]]:
    
    vectors = []
    for x in range(0, w, step):
        for y in range(0, h, step):
            if obstacles is None:
                vector = univector_field.Nh(
                    get_vec.compute(
                        [x/100, y/100], util.wrap_to_pi(math.radians(0))
                    )
                )
            else:
                pass
                #not implemented
            vectors.append(vector)

    return vectors