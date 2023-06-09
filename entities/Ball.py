'''
Created by: - Luís Henrique
            - Lucas

Date: 15/06/2023
'''

from .Entity import Entity
from geometry.Point import Point
from vision.vision import Vision

class Ball(Entity):
    """
    Contains the ball state
    """

    def __init__(
        self,
        position: Point = Point(0,0),
        velocity: float = 0,
    ) -> None:
        super().__init__(position, velocity)

    def update(self, game) -> None:
        """
        Updates the ball own state
        """
        frame = game.update()

        ball_frame = frame['ball']
        self.position = Point(
            ball_frame.get('x', 0),
            ball_frame.get('y', 0)
        )
        self.linear_velocity = 0
