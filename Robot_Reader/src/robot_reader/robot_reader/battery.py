# Copyright © 2023 University of Turin, Daniele Bortoluzzi & Giordano Scarso. All Rights Reserved.

import string

class BatteryDTO:
    """!
    Class used to store battery sensor data.
    """
    def __init__(self, robot: string, percentage_charge: float = None, temperature: float = None, capacity: float = None, power_supply_status: int = None):
        self.robot                  = robot
        self.percentage_charge      = percentage_charge
        self.temperature            = temperature
        self.capacity               = capacity
        self.power_supply_status    = power_supply_status

    def __str__ (self):
        return 'BatteryDTO(robot={0}, percentage_charge={1}, temperature={2}, capacity={3}, power_supply_status={4})'.format(
                self.robot, str(self.percentage_charge), str(self.temperature),
                str(self.capacity), str(self.power_supply_status))
