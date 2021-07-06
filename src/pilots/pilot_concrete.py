# libraries for class structure
from __future__ import annotations
from abc import ABC, abstractmethod
import struct
import sys
import keyboard

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
    def send_command(self):
        # abstract method for pilot to prepare command
        pass


# ------CONCRETE CLASS DEFINITIONS----------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
class Autopilot1(AutoPilot):
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
    _padding = "zip"

    def __init__(self):
        self._lateral_airspeed = 0
        self._drop_package_commanded = 0
        self._drop_timestamp = 0 
        self._timestamp = 0

    # skeleton class for manualpilot implementation
    def interpret_telemetry(self, telemetry_buffer):
        #save instance of timestamp
        telemetry = TELEMETRY_STRUCT.unpack(telemetry_buffer)
        self._timestamp = telemetry[0]
        return None

    def send_command(self):
        self._lateral_airspeed -= self._lateral_airspeed / 0.5 * (1 / 60.0)
        if keyboard.is_pressed("a"):
            self._lateral_airspeed = min(
                30.0, self._lateral_airspeed + (1 / 60.0) * 200.0
            )
        if keyboard.is_pressed("d"):
            self._lateral_airspeed = max(
                -30.0, self._lateral_airspeed - (1 / 60.0) * 200.0
            )
        if keyboard.is_pressed("space"):
            time_elapsed = self._timestamp - self._drop_timestamp
            if time_elapsed > 500:
                self._drop_package_commanded = 1
                self._drop_timestamp = self._timestamp

        cmd = COMMAND_STRUCT.pack(
            self._lateral_airspeed,
            self._drop_package_commanded,
            self._padding.encode(),
        )

        # send command back to parent process
        try:
            sys.stdout.buffer.write(cmd)
            sys.stdout.flush()
        except struct.error as e:
            raise e

        self._drop_package_commanded = 0

        return None
