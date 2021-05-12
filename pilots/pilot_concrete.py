# libraries for class structure
from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Type
import struct
import sys
import serial
import time

from .controllers.controller_creator import AutoControlCreator
from zip_sim import TELEMETRY_STRUCT, COMMAND_STRUCT
from .config import arduino_nano

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

    def __init__(self):
        self.ctrl = AutoControlCreator()

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

    def interpret_telemetry(self, telemetry_struct):
        # method for autopilot1 to interpret telemetry data
        self.ctrl.receive_data(TELEMETRY_STRUCT.unpack(telemetry_struct))


class ArduinoPilot(AutoPilot):
    # boilerplace code for class to represent different potential autopilot configurations
    # could be swapped during flight or selected based on flight conditions
    # idk just a thought! :-)

    def __init(self):
        self.arduino = serial.Serial(
            port=arduino_nano['PORT'], baudrate=arduino_nano['BAUD_RATE'])

    def interpret_telemetry(self, buffer):
        self.arduino.write(buffer)

    def send_command(self):
        buf = self.arduino.readline()
        if(buf == COMMAND_STRUCT.size):
            try:
                sys.stdout.buffer.write(buf)
                sys.stdout.flush()
            except struct.error as e:
                raise e
            return None

    def pid_control(self):
        # skeleton method for autopilot pid

        # self.pid = PID(1, 0.1, 0.05, setpoint=1)
        # self.col_pid = PID(-1.0, -0.1, 0)
        # self.pid.sample_time = 0.01  # Update every 0.01 seconds
        # self.col_pid.sample_time = 0.01  # Update every 0.01 seconds
        return None


class ManualPilot1(ManualPilot):
    # skeleton class for manualpilot implementation
    def interpret_telemetry(self):
        # skeleton method for pilot to interpret telemetry received
        return None

    def prepare_command(self):
        # abskeletonstract method for pilot to prepare command
        return None
