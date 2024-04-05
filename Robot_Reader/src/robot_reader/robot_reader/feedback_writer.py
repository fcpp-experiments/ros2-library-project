# Copyright © 2023 University of Turin, Daniele Bortoluzzi & Giordano Scarso. All Rights Reserved.

from .position   import PositionDTO
from .battery    import BatteryDTO
from .goal       import GoalDTO
from .dock       import DockDTO

import string, time, csv
from datetime import datetime
from pathlib import Path
import os

STORAGE_BASE_PATH           = "../../../../Storage/"
ROBOT_PLACEHOLDER           = "#ROBOT"
ROBOT_OUTPUT_PATH_TO_WEB    = "from_robot/#ROBOT/to_web/feedback/"
ROBOT_OUTPUT_PATH_TO_AP     = "from_robot/#ROBOT/to_ap/feedback/"
DELIMITER                   = ";"

class FeedbackWriter:
    """!
    Class responsible for writing feedback files.
    """
    def __init__(self):
        pass

    def write_file(self, robot:string, position:PositionDTO, battery:BatteryDTO,
            goal:GoalDTO, dock:DockDTO, suffix=""):
        """!
        Given a robot name, position data, battery data and goal data, write them into
        a feedback file, eventually creating missing directories.
        """

        dt_obj = datetime.now()  # input datetime object
        millis = int(float(dt_obj.strftime('%s.%f')) * 1e3)
        filename = "feedback-{}{}.txt".format(millis, suffix)
        
        path_web = STORAGE_BASE_PATH + ROBOT_OUTPUT_PATH_TO_WEB.replace(ROBOT_PLACEHOLDER, robot) + filename
        path_ap = STORAGE_BASE_PATH + ROBOT_OUTPUT_PATH_TO_AP.replace(ROBOT_PLACEHOLDER, robot) + filename

        data = [robot,
                position.pos_x,
                position.pos_y,
                position.angle,
                battery.percentage_charge,
                goal.goal_status,
                goal.goal_id,
                goal.current_step,
                dock.goal_status,
                1, #TODO: implement system_status, now it's 1 (OK)
                millis]

        path_web_abs = Path(__file__).parent / path_web
        path_ap_abs = Path(__file__).parent / path_ap
        path_web_abs.parent.mkdir(exist_ok=True, parents=True)
        path_ap_abs.parent.mkdir(exist_ok=True, parents=True)

        with path_web_abs.with_suffix('.lock').open(mode="w") as file_to_web:
            writer = csv.writer(file_to_web, delimiter=DELIMITER, quoting=csv.QUOTE_NONE)
            writer.writerow(data)
        os.rename(path_web_abs.with_suffix('.lock'), path_web_abs)

        with path_ap_abs.with_suffix('.lock').open(mode="w") as file_to_ap:
            writer = csv.writer(file_to_ap, delimiter=DELIMITER, quoting=csv.QUOTE_NONE)
            writer.writerow(data)
        os.rename(path_ap_abs.with_suffix('.lock'), path_ap_abs)

        return filename
