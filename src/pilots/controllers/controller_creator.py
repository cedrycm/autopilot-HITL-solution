from __future__ import annotations
from abc import ABC, abstractmethod
from enum import IntFlag
import serial
import time
import struct
import sys

from .controller_components import SpeedController, PackageController
from zip_sim import TELEMETRY_STRUCT, COMMAND_STRUCT
from config import arduino

ARDUINO_COMMAND_STRUCT = struct.Struct("<fB3s")  # struct for little endian conversion

WORLD_LENGTH = 2000.0

VEHICLE_AVOID_THRESHOLD = 30

VEHICLE_WINGSPAN_RADIUS = 1.6

TREE_RADIUS = 3.0

# 0.5 buffer for average diameter values
LIDAR_DELIVERY_DIAMETER = 1.5

LAT_AVOIDANCE_DISTANCE = VEHICLE_WINGSPAN_RADIUS + TREE_RADIUS

VEHICLE_AIRSPEED = 30.0
# How long it takes for the package to "fall" and hit the ground after being released.
PACKAGE_FALL_SEC = 0.5

# ------Controller Flags--------------------------------------------------------
# ------------------------------------------------------------------------------
class PilotFlags(IntFlag):
    APPROACH_TARGET = 0
    AVOID_COLLISION = 1
    RECOVER = 2


# ------Creator  Class Definitions----------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
class ControllerCreator(ABC):
    # abstract decorator for controller creation method
    @abstractmethod
    def create_controller(self):
        pass


class AutoControlCreator(ControllerCreator):
    @classmethod
    def create_controller(self, controller_select="AUTO"):
        if controller_select == "AUTO":
            speed_controller = SpeedController()
            package_controller = PackageController()
            return AutoController1(speed_controller, package_controller)
        elif controller_select == "UNO":
            return ArduinoController()


# ------CONCRETE CLASS DEFINITIONS----------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
class AutoController(ABC):
    @abstractmethod
    def receive_data(self):
        pass

    @abstractmethod
    def return_data(self):
        pass


class AutoController1(AutoController):
    __slots__ = ["_flag_status", "_v_y", "_d_x_last", "_d_y_last"]

    def __init__(self, speed_controller, package_controller):
        self._flag_status = PilotFlags.APPROACH_TARGET
        self._speed_ctrl = speed_controller
        self._package_ctrl = package_controller
        # self._d_rel = None

        super().__init__()

    def receive_data(self, telemetry_buffer):
        # method for autopilot1 to interpret telemetry data
        telemetry = TELEMETRY_STRUCT.unpack(telemetry_buffer)

        timestamp = telemetry[0]
        recovery_x_error = telemetry[1]
        wind_vector_x = float(telemetry[2])
        wind_vector_y = float(telemetry[3])
        recovery_y_error = telemetry[4]
        lidar_samples = list(telemetry[5:])[::-1]

        # update the speed controller
        self._speed_ctrl.speed_inputs(wind_vector_x, wind_vector_y, lidar_samples)

        if recovery_x_error < 100.00 or self._flag_status == PilotFlags.RECOVER:
            self._flag_status = PilotFlags.RECOVER
            self._speed_ctrl.update_airspeed(
                distance=(recovery_x_error, recovery_y_error)
            )

        elif (
            # if object is diameter is larger that lidar_delivery_diameter check if collision avoidance is needed
            self._speed_ctrl.d_1_2 != None
            and self._speed_ctrl.d_1_2 > LIDAR_DELIVERY_DIAMETER
        ):
            if (
                self._speed_ctrl.distance[0] < VEHICLE_AVOID_THRESHOLD
                and self._speed_ctrl.distance[1] < LAT_AVOIDANCE_DISTANCE
            ):
                self._flag_status = PilotFlags.AVOID_COLLISION
                # change target distance to be away from tree collision boundary
                self._speed_ctrl.avoid_collision(lidar_samples)
                self._speed_ctrl.update_airspeed()
        elif self._speed_ctrl.d_1_2 != None:
            self._flag_status = PilotFlags.APPROACH_TARGET
            self._speed_ctrl.update_airspeed()

            v_x_sum = VEHICLE_AIRSPEED + wind_vector_x
            v_y_sum = self._speed_ctrl.v_y + wind_vector_y

            """Issue where packages were being dropped to trees on the corner of the lidar scans
            set lidar boundary to not drop if the object is located at the edges of vehicle bounds"""
            # TODO: adjust lateral speed with proportion to distance from the center of lidar scanner
            if abs(self._speed_ctrl.theta) < 10.0:
                # update the package controller to check for drop
                self._d_rel = (v_x_sum * PACKAGE_FALL_SEC, v_y_sum * PACKAGE_FALL_SEC)
                self._package_ctrl.update_target_params(
                    position=self._speed_ctrl._distance,
                    current_drop_pos=self._d_rel,
                    current_time=timestamp,
                )
            elif abs(self._speed_ctrl.theta) < 12.0:
                # increase the speed by 20% if target is on the edges of the lidar boundary
                self._speed_ctrl._v_y = self._speed_ctrl.v_y * 1.2
            elif abs(self._speed_ctrl.theta) > 12.0:
                # increase the speed by 20% if target is on the edges of the lidar boundary
                self._speed_ctrl._v_y = self._speed_ctrl.v_y * 1.5

    def return_data(self):
        return (self._speed_ctrl.v_y, self._package_ctrl.drop_status)


class ArduinoController(AutoController):
    def __init__(self):
        self._arduino = serial.Serial(
            arduino["port"], timeout=arduino["timeout"], baudrate=arduino["baud"]
        )
        time.sleep(1)
        self._telemetry_buffer = None
        self._emergency_counter = 0

    def receive_data(self, telemetry_buffer):
        if telemetry_buffer != None:
            # store telemetry for emergency case
            self._telemetry_buffer = telemetry_buffer
            self.__send_packet(telemetry_buffer)

    def return_data(self):
        """Interface for reading data out of arduino controller and pipeing it
        back into zip_sim"""

        payload = None
        start_time = time.time()
        while payload == None and time.time() < (start_time + arduino["timeout"]):
            payload = self.__read_packet()

        if payload != None:
            (lateral_airspeed, drop_flag, _) = ARDUINO_COMMAND_STRUCT.unpack(payload)

        else:
            (lateral_airspeed, drop_flag) = self.__emergency_command()

        return (lateral_airspeed, drop_flag)

    def __send_packet(self, buffer):
        payload = None
        tx = b"\x10\x02"  # start sequence
        tx += struct.pack("<B", 44)  # length of data
        tx += bytes(buffer)
        tx += struct.pack("<B", self.__calc_checksum(buffer))
        tx += b"\x10\x03"  # end sequence
        self._arduino.write(tx)
        # start_time = time.time()
        # while payload == None and time.time() < (start_time + arduino["timeout"]):
        #     payload = self.__read_packet()

        # end_time = time.time()
        # tot = end_time - start_time
        # str_time = "Round Trip Time: " + str(tot)

        # return None

    def __emergency_command(self):
        """Returns emergency command to negate lateral wind velocity if
        arduino connection fails

        returns empty after arbitrary 10 tries"""

        if self._emergency_counter < 5:
            telemetry = TELEMETRY_STRUCT.unpack(bytes(self._telemetry_buffer))
            v_y = float(telemetry[3]) * -1
            self._emergency_counter += 1

            return (v_y, 0)
        else:
            return None

    def __calc_checksum(self, data):
        calculated_checksum = 0
        for byte in data:
            calculated_checksum ^= byte
        return calculated_checksum

    def __read_packet(self):
        """
        :return received data in the packet if read sucessfully, else return None
        """
        try:
            # check start sequence
            if self._arduino.read() != b"\x10":
                return None

            if self._arduino.read() != b"\x02":
                return None

            payload_len = self._arduino.read()[0]
            if payload_len != ARDUINO_COMMAND_STRUCT.size:
                # could be other type of packet, but not implemented for now
                return None

            # we don't know if it is valid yet
            payload = self._arduino.read(payload_len)

            checksum = self._arduino.read()[0]
            if checksum != self.__calc_checksum(payload):
                return None  # checksum error

            # check end sequence
            if self._arduino.read() != b"\x10":
                return None
            if self._arduino.read() != b"\x03":
                return None
            # yeah valid packet received
            return payload
        except self._arduino.SerialTimeoutException:
            sys.stderr.write("Controller has timed out.\n")
            raise TimeoutError
