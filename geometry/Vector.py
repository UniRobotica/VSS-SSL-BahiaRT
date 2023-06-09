'''
Created by: - Luís Henrique
            - Lucas

Date: 15/06/2023
'''

import numpy as np
import math

class Vector:
    """
    Represents a vector in a 2D space
    """
    def __init__(self, x: float = 0, y: float =0 ) -> None:
        self.x = x
        self.y = y
    
    def __str__(self):
        return f'x: {self.x:.02f}  y: {self.y:.02f}'

    def __repr__(self):
        return f'Vector({self})'
    
    def unit_vector(self) -> list[float]:
        """
        Calculates the vector normalization and, from that, calculates the unit vector.

        Returns:
            list[float]: An unit vector [x, y]
        """
        
        if np.linalg.norm(self.values()) == 0:
            return np.array([0, 0])
        return self.values() / np.linalg.norm(self.values())
    
    def setValues(self, x: float, y: float):
        self.x = x
        self.y = y

    def setValues(self, vector: list[float]):
        self.x = vector[0]
        self.y = vector[1]
        
    def values(self):
        return[
            self.x,
            self.y
        ]

    def getMagnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def normalize(self):
        magnitude = self.getMagnitude()
        if magnitude == 0:
            raise ZeroDivisionError("Cannot normalize a zero vector.")
        return Vector(self.x / magnitude, self.y / magnitude)

    def dotProduct(self, other: "Vector"):
        if not isinstance(other, Vector):
            raise TypeError("Invalid Vector type")
        return self.x * other.x + self.y * other.y

    def crossVectors(self, other: "Vector"):
        if not isinstance(other, Vector):
            raise TypeError("Invalid Vector type")
        return self.x * other.y - self.y * other.x

    def directionTo(self, other: "Vector"):
        if not isinstance(other, Vector):
            raise TypeError("Invalid Vector type")
        direction = Vector(other.x - self.x, other.y - self.y)
        return direction.normalize()

    def rotationTo(self, other: "Vector", orientation: float):
        if not isinstance(other, Vector):
            raise TypeError("Invalid Vector type")
        direction = self.directionTo(other)
        dot_product = self.dotProduct(direction)
        angle = math.acos(dot_product)
        if self.crossVectors(direction) < 0:
            angle *= -1
        return angle - orientation

    def rotationTo(self, other: "Vector", orientation: float):
        if not isinstance(other, Vector):
            raise TypeError("Invalid Vector type")
        dot_product = self.dotProduct(other)
        angle = math.acos(dot_product / (self.getMagnitude() * other.getMagnitude()))
        if self.crossVectors(other) < 0:
            angle *= -1
        return angle - orientation

    def toString(self):
        return f"({self.x}, {self.y})"


# Usage:
if __name__ == "__main__":
    vector1 = Vector(1.0, 2.0)
    vector2 = Vector(3.0, 4.0)
    magnitude = vector1.getMagnitude()
    normalized_vector = vector2.normalize()
    dot_product = vector1.dotProduct(vector2)
    cross_product = vector1.crossVectors(vector2)
    print(f"Magnitude: {magnitude}")
    print(f"Normalized Vector: {normalized_vector.toString()}")
    print(f"Dot Product: {dot_product}")
    print(f"Cross Product: {cross_product}")