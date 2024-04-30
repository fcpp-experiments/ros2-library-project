# Copyright © 2023 University of Turin, Daniele Bortoluzzi & Giordano Scarso. All Rights Reserved.

import string

goal_status_code = [
        'Reached',
        'Aborted',
        'Failed',
        'Running',
        'Unknown error',
        'No goal']

class GoalDTO:
    """!
    Class storing the current goal state of the robot,
    all the possible value of goal_status are in goal_status_code.
    """
    def __init__(self, robot:string, goal_id:string = "", goal_status: int = -1, current_step = 0):
        self.goal_id        = goal_id
        self.goal_status    = goal_status
        self.current_step   = current_step
        self.robot          = robot

    def __str__ (self):
        return 'GoalDTO(robot={0}, goal_id={1}, goal_status={2}, current_step={3})'.format(
                self.robot,
                self.goal_id,
                goal_status_code[self.goal_status],
                self.current_step)
