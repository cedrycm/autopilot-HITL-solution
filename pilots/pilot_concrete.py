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
        self.telemetry_buffer = None
        self.emergency_counter = 0
        self.timeout = 3
        self.arduino = serial.Serial(
            port=arduino_nano["PORT"], baudrate=arduino_nano["BAUD_RATE"]
        )
        time.sleep(3)

    def interpret_telemetry(self, buffer):
        # store telemetry for emergency case
        self.telemetry_buffer = buffer
        self.__send_packet(buffer)

    def send_command(self):
        """Interface for reading data out of arduino controller and pipeing it
        back into zip_sim"""

        payload = None
        start_time = time.time()
        while payload == None and time.time() < (start_time + self.timeout):
            payload = self.__read_packet()

        if payload != None:
            try:
                sys.stdout.buffer.write(payload)
                sys.stdout.flush()
            except struct.error as e:
                raise e
            return None
        else:
            payload = self.__emergency_command()

            if payload == None:
                return None

    def __send_packet(self, buffer):
        tx = b"\x10\x02"  # start sequence
        tx += struct.pack(">B", TELEMETRY_STRUCT.size)  # length of data
        tx += buffer
        tx += struct.pack(">B", self.__calc_checksum(buffer))
        tx += b"\x10\x03"  # end sequence
        print("Sending:", tx.hex())
        self.arduino.write(tx)

    def __emergency_command(self):
        """Returns emergency command to negate lateral wind velocity if
        arduino connection fails

        returns empty after arbitrary 10 tries"""

        if self.emergency_counter < 10:
            telemetry = TELEMETRY_STRUCT.unpack(self.telemetry_struct)
            wind_vector_y = float(telemetry[3])
            self.emergency_counter += 1

            return COMMAND_STRUCT.pack(0, -wind_vector_y, "zip")
        else:
            return None

    def __calc_checksum(data):
        calculated_checksum = 0
        for byte in data:
            calculated_checksum ^= byte
        return calculated_checksum

    def __read_packet(self):
        """Return received data in the packet if read sucessfully, else return None"""
        # check start sequence
        if self.arduino.read() != b"\x10":
            return None

        if self.arduino.read() != b"\x02":
            return None

        payload_len = self.arduino.read()[0]
        if payload_len != COMMAND_STRUCT.size:
            # could be other type of packet, but not implemented for now
            return None

        # we don't know if it is valid yet
        payload = self.arduino.read(payload_len)

        checksum = self.arduino.read()[0]
        if checksum != self.__calc_checksum(payload):
            return None  # checksum error

        # check end sequence
        if self.arduino.read() != b"\x10":
            return None
        if self.arduino.read() != b"\x03":
            return None

        # valid packet received
        return payload


class ManualPilot1(ManualPilot):
    # skeleton class for manualpilot implementation
    def interpret_telemetry(self):
        # skeleton method for pilot to interpret telemetry received
        return None

    def prepare_command(self):
        # abskeletonstract method for pilot to prepare command
        return None
