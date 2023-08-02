'''
Created by: - Luís Henrique
            - Lucas

Date: 15/06/2023
'''

from vision.vision import Vision

class Ball():
    """
    Contains the ball state
    """

    def __init__(
        self,
        position: list[float] = [],
        velocity: list[float] = [],
    ) -> None:
        self.position = position
        self.velocity = velocity

    def update(self, frame) -> None:
        """
        Updates the ball own state
        """

        ball_frame = frame.get('detection')['balls'][0]
        self.position = [
            ball_frame.get('x', 0),
            ball_frame.get('y', 0)
        ]
