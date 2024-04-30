# Copyright © 2023 University of Turin, Daniele Bortoluzzi & Giordano Scarso. All Rights Reserved.

import string

class PositionDTO:
    """!
    Class used to represent robot position, measures x, y are in meters.
    angle value is expressed in radians.
    """
    def __init__(self, robot: string, pos_x: float = None, pos_y: float = None,
            angle: float = None):
        self.robot      = robot
        self.pos_x      = pos_x
        self.pos_y      = pos_y
        self.angle   = angle

    def __str__ (self):
        return """PositionDTO(robot={0}, pos_x={1}, pos_y={2}, angle={3})""".format(
                self.robot, str(self.pos_x), str(self.pos_y), str(self.angle))
