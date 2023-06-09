'''
Created by: - Luís Henrique
            - Lucas

Date: 30/06/2023
'''

import math

class Point():
    """
    Represents a point in a 2D space
    """
    def __init__(self, x: float = 0, y: float = 0) -> None:
        self.x = x
        self.y = y
    
    def __str__(self):
        return f"x:{self.x}, y:{self.y}"
    
    def __repr__(self):
        return f'Point({self})'
    
    def __neg__(self):
        return Point(-self.x, -self.y)

    def __sub__(self, otherPoint):
        if not isinstance(otherPoint, Point):
            raise RuntimeError("Invalid Point type")
        dx = self.x - otherPoint.x
        dy = self.y - otherPoint.y
        return Point(dx, dy)
        
    def values(self) -> list[float]:
        """
        Gives the point values

        Returns:
            list[float]: an array [x,y]
        """
        return [
            self.x, 
            self.y
        ]
    
    def distanceTo(self, point):
        if not isinstance(point, Point):
            raise RuntimeError("Invalid Point type")
        dx = self.x - point.x
        dy = self.y - point.y
        return math.sqrt(dx ** 2 + dy ** 2)

    def calculateAngularCoefficient(self, otherPoint: "Point"):
        if not isinstance(otherPoint, Point):
            raise RuntimeError("Invalid Point type")
        dx = self.x - otherPoint.x
        dy = self.y - otherPoint.y
        if dx == 0:
            raise RuntimeError("Division by zero")
        return dy / dx

    def calculateLinearCoefficient(self, otherPoint):
        if not isinstance(otherPoint, Point):
            raise RuntimeError("Invalid Point type")
        dx = self.x - otherPoint.x
        dy = self.y - otherPoint.y
        if dx == 0:
            raise RuntimeError("Division by zero")
        return (self.y * otherPoint.x - self.x * otherPoint.y) / dx

    def calculateInverseTangent(self, otherPoint: "Point"):
        if not isinstance(otherPoint, Point):
            raise RuntimeError("Invalid Point type")
        dx = self.x - otherPoint.x
        dy = self.y - otherPoint.y
        return math.atan2(dy, dx)

    def doesBelongToLine(self, lineEnd1, lineEnd2, errorMargin=0.0):
        if not isinstance(lineEnd1, Point) or not isinstance(lineEnd2, Point):
            raise RuntimeError("Invalid Point type")
        x1 = lineEnd1.x
        y1 = lineEnd1.y
        x2 = lineEnd2.x
        y2 = lineEnd2.y
        x = self.x
        y = self.y
        distance = ((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return abs(distance) <= errorMargin

    def rotate(self, angleInRadians, anchorPoint: "Point"):
        if not isinstance(anchorPoint, Point):
            raise RuntimeError("Invalid Point type")
        dx = self.x - anchorPoint.x
        dy = self.y - anchorPoint.y
        cos_theta = math.cos(angleInRadians)
        sin_theta = math.sin(angleInRadians)
        x = anchorPoint.x + dx * cos_theta - dy * sin_theta
        y = anchorPoint.y + dx * sin_theta + dy * cos_theta
        return Point(x, y)

    def isInFrontOf(self, x):
        return self.x > x

    def isBehindOf(self, x):
        return self.x < x

    def isInBetween(self, objectStart, objectFinal):
        if not isinstance(objectStart, Point) or not isinstance(objectFinal, Point):
            raise RuntimeError("Invalid Point type")
        return (
            min(objectStart.x, objectFinal.x) <= self.x <= max(objectStart.x, objectFinal.x)
            and min(objectStart.y, objectFinal.y) <= self.y <= max(objectStart.y, objectFinal.y)
        )