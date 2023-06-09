'''
Created by: - Luís Henrique
            - Lucas

Date: 30/06/2023
'''

class Pose:
    def __init__(self, x: float = 0, y: float = 0, theta: float = 0):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return f'x: {self.x:.02f}  y: {self.y:.02f}  th: {self.theta:.02f}'

    def __repr__(self):
        return f'Pose({self})'
    
    def getPoseValues(self):
        return [self.x, self.y, self.theta]
    
    def getPointValues(self):
        return [self.x, self.y]
