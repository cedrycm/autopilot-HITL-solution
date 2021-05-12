# libraries for class structure
from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Type

# concrete classes
from .pilot_concrete import ArduinoPilot, Autopilot1, ManualPilot1

from .controllers.controller_creator import AutoControlCreator


# # ------Vehicle Pilot Selection Functions--------------------------------------------
# # -----------------------------------------------------------------------------------
# # -----------------------------------------------------------------------------------
# Import functions below to create concrete pilot classes


def select_autopilot(autopilot_id: int):
    return AutoPilotCreator.create_pilot(autopilot_id)


def select_manualpilot(pilot_id: int):
    return ManualPilotCreator.create_pilot(pilot_id)


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
    # 1: Path Kinematics-based Autopilot Controller
    # 2: TODO: Experimental PID for pilot_concrete.autopilot2 class?
    @classmethod
    def create_pilot(self, autopilot_id: int):
        if autopilot_id == 1:
            controller = AutoControlCreator.create_controller()
            return Autopilot1(controller)
        elif autopilot_id == 2:
            return ArduinoPilot() #Arduino pilot


class ManualPilotCreator(PilotCreator):
    # skeleton class for creating different types of pilots
    # ManualPilot(MP) Class creates pilot
    @classmethod
    def create_pilot(self, pilot_id: int):
        if pilot_id == 1:
            return ManualPilot1()


# remove comment below for debuging autopilot class instance
# if __name__ == "__main__":
#     # create instance of our defined 'autopilot1' class
#     test1 = select_pilot(AutoPilotCreator.create_pilot(1))
#     pilot = select_autopilot(1)