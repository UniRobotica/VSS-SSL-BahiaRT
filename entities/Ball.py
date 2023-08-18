'''
Created by: - Luís Henrique
            - Lucas

Date: 15/06/2023
'''

from collections import deque
from utils import speed

class Ball():
    """
    Contains the ball state
    """

    def __init__(
        self,
        env='simulation'
    ) -> None:
        
        self.position = [.0,.0]
        self.vx, self.vy = .0 , .0
        
        self.last_positions = {
            'x': deque(maxlen=10),
            'y': deque(maxlen=10),
        }
        
        self.env = env
        
    def _update_last_positions(self):
        
        self.last_positions['x'].append(self.position[0])
        self.last_positions['y'].append(self.position[1])
    
    def _update_speed(self):
        
        self._update_last_positions()
        self.vx = speed.calculate_linear_speed(self.last_positions['x'])
        self.vy = speed.calculate_linear_speed(self.last_positions['y'])
        
    def update(self, frame) -> None:
        """
        Updates the ball own state
        """
        
        if self.env == 'real':
            ball_frame = frame.get('detection')['balls'][0]
        else:
            ball_frame = frame['ball']
        self.position = [
            ball_frame.get('x', 0),
            ball_frame.get('y', 0)
        ]
        
        self._update_speed()
