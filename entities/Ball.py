'''
Created by: - Luís Henrique
            - Lucas

Date: 15/06/2023
'''

class Ball():
    """
    Contains the ball state
    """

    def __init__(
        self,
        env='simulation'
    ) -> None:
        
        self.position = 0
        self.velocity = 0
        self.env = env
        
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
