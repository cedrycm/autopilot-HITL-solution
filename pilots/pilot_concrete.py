# libraries for class structure
from __future__ import annotations
from abc import ABC, abstractmethod
import struct
import sys
import time

from zip_sim import TELEMETRY_STRUCT, COMMAND_STRUCT

# ----------BASE CLASS DEFINITIONS----------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
class Pilot(ABC):
    @abstractmethod
    def interpret_telemetry(self):
        # abstract method for pilot to interpret telemetry received
        pass

    @abstractmethod
    def send_command(self):
        # abstract method for pilot to prepare command
        pass


class AutoPilot(Pilot):
    @abstractmethod
    def interpret_telemetry(self):
        # abstract method for pilot to interpret telemetry received
        pass

    @abstractmethod
    def send_command(self):
        # abstract method for pilot to prepare command
        pass


class ManualPilot(Pilot):
    @abstractmethod
    def interpret_telemetry(self):
        # abstract method for pilot to interpret telemetry received
        pass

    @abstractmethod
    def prepare_command(self):
        # abstract method for pilot to prepare command
        pass


# ------CONCRETE CLASS DEFINITIONS----------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
class Autopilot1(AutoPilot):
    __slots__ = []
    _padding = "zip"

    def __init__(self, controller):
        self.ctrl = controller

    def send_command(self):
        # retrieve data from controller
        (v_y, drop_status) = self.ctrl.return_data()

        # create tuple for command
        cmd = COMMAND_STRUCT.pack(
            v_y,
            drop_status,
            self._padding.encode(),
        )

        # send command back to parent process
        try:
            sys.stdout.buffer.write(cmd)
            sys.stdout.flush()
        except struct.error as e:
            raise e
        return None

    def interpret_telemetry(self, telemetry_buffer):
        # method for autopilot1 to interpret telemetry data
        self.ctrl.receive_data(telemetry_buffer)
        return None


class ManualPilot1(ManualPilot):
    # skeleton class for manualpilot implementation
    def interpret_telemetry(self):
        # skeleton method for pilot to interpret telemetry received
        return None

    def prepare_command(self):
        # abskeletonstract method for pilot to prepare command
        return None
