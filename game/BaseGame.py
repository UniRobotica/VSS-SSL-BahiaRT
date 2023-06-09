'''
Created by: - Luís Henrique
            - Lucas

Date: 01/06/2023
'''

from abc import ABC, abstractmethod

import time

class BaseGame(ABC):
    
    def __init__(
        self,
        vision,
    ) -> None:
        self.vision       = vision
        self.game_config  = None
        self.field        = None
        self.entities     = None
        self.time         = 0.0
        self.start_time   = time.time()
    
    def _game_time(self) -> None:
        self.time = time.time() - self.start_time
    
    @abstractmethod
    def _field_info(self) -> dict:
        pass
    
    @abstractmethod
    def _entities_info(self) -> dict:
        pass
    
    @abstractmethod
    def update(self):
        pass