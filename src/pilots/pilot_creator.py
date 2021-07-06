# libraries for class structure
from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Type

import serial

# concrete classes
from .pilot_concrete import Autopilot, ManualPilot

from .controllers.controller_creator import AutoControlCreator


# # ------Vehicle Pilot Selection Functions--------------------------------------------
# # -----------------------------------------------------------------------------------
# # -----------------------------------------------------------------------------------
# Import functions below to create concrete pilot classes
class PilotDirector:
    @classmethod
    def select_pilot(self, pilot_id: str):
        if pilot_id == "MANUAL":
            pilotCreator = ManualPilotCreator(pilot_id)
            return pilotCreator.create_pilot()
        else:
            pilotCreator = AutoPilotCreator(pilot_id)
            return pilotCreator.create_pilot()


# ------CREATOR AND BASE CLASS DEFINITIONS----------------------------------------------
# --------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------
class PilotCreator(ABC):
    # contains base abstract method(s) for necessary pilot creator behavior
    @abstractmethod
    def create_pilot(self):
        # factory method - creates different types of pilots
        pass


class AutoPilotCreator(PilotCreator):
    # build and return an AutoPilot(AP) class
    # AP IDs:
    # 1: AUTO: Path Kinematics-based Autopilot Controller
    # 2: UNO: Arduino Autopilot MicroController over serial bus
    def __init__(self, pilot_id) -> None:
        super().__init__()
        self.pilot_id = pilot_id
    def create_pilot(self):
        controller = AutoControlCreator.create_controller(self.pilot_id)
        return Autopilot(self.pilot_id, controller)


class ManualPilotCreator(PilotCreator):
    # skeleton class for creating different types of pilots
    # ManualPilot(MP) Class creates pilot
    # 1: MANUAL: implements controller interrupts to control lateral velocity & drop
    def __init__(self, pilot_id) -> None:
        super().__init__()
        self.pilot_id = pilot_id

    def create_pilot(self):
        return ManualPilot(self.pilot_id)


# remove comment below for debuging autopilot class instance
# if __name__ == "__main__":
#     # create instance of our defined 'autopilot1' class
#     test1 = select_pilot(AutoPilotCreator.create_pilot(1))
#     pilot = select_autopilot(1)
