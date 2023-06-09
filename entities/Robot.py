'''
Created by: - Luís Henrique
            - Lucas

Date: 30/06/2023
'''

from .Entity import Entity
from geometry.Point import Point
from geometry.Vector import Vector
from geometry.Pose import Pose
from game.Game import Game

class Robot(Entity):
    '''
    A base class that contains the state of the robot and basic movement methods for all game positions.
    '''
    # Robot length(L) and radius(R)
    L = 0.075
    R = 0.02

    def __init__(
        self,
        game: Game,
        position: Pose = Pose(),
        velocity: Vector = Vector(),
        robot_id: int = 0,
        team_color: bool = True # True: blue_team | False: yellow_team
    ) -> None:
        super().__init__(position, velocity)
        self.game = game
        self.robot_id = robot_id
        self.team_color = team_color

    def team_color_str(self) -> str:
        if self.team_color:
            return 'blue'
        else:
            return 'yellow'

    def update(self) -> None:
        """
        Updates the robot's own state
        """
        if self.team_color:
            _team_color = "robotsBlue"
        else:
            _team_color = "robotsYellow"

        info = self.game.blueRobot_info(self.robot_id)
        self.position = Pose(
            info.get('x', 0),
            info.get('y', 0),
            info.get('orientation', 0)
        )
        self.linear_velocity = Vector(0,0)
        
    def printInfo(self):
        print('-----------------------------')
        print(self.position)
        print(self.velocity)
        print('-----------------------------')