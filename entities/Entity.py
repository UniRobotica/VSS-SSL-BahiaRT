'''
Created by: - Luís Henrique
            - Lucas

Date: 30/06/2023
'''

from geometry.Pose import Pose
from geometry.Vector import Vector

class Entity(object):
    def __init__(self, position: Pose = Pose(), velocity: Vector = Vector()):
        self.position = position
        self.velocity = velocity

    def __str__(self):
        return (
            f'Position: {self.position}\n'
            f'Velocity: {self.velocity}\n'
        )

    def __repr__(self):
        return f'Entity({self})'