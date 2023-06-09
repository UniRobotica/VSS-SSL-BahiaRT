'''
Created by: - Luís Henrique
            - Lucas

Date: 01/06/2023
'''

from .BaseGame import BaseGame
from vision.vision import Vision

class Game(BaseGame):
    
    def __init__(self, vision: Vision) -> None:
        super().__init__(vision)
    
    def _field_info(self) -> dict:
        return self.vision.frame.get('geometry')
    
    def _entities_info(self) -> dict:
        return self.vision.frame.get('detection')
    
    def blueRobot_info(self, id):
        blue_entities = self._entities_info().get('robotsBlue')
        for blueRobot in blue_entities:
            if blueRobot.get('robotId') == id:
                return blueRobot
    
    def update(self):
        return 0
    
    