from __future__ import annotations
from abc import ABC, abstractmethod
from .controller_components import SpeedController, PackageController
from enum import IntFlag

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
    def create_controller(self):
        speed_controller = SpeedController()
        package_controller = PackageController()
        return AutoController1(speed_controller, package_controller)


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
        self.speed_ctrl = speed_controller
        self.package_ctrl = package_controller
        # self._d_rel = None

        super().__init__()

    def receive_data(self, telemetry):
        # method for autopilot1 to interpret telemetry data
        timestamp = telemetry[0]
        recovery_x_error = telemetry[1]
        wind_vector_x = float(telemetry[2])
        wind_vector_y = float(telemetry[3])
        recovery_y_error = telemetry[4]
        lidar_samples = list(telemetry[5:])[::-1]

        # update the speed controller
        self.speed_ctrl.speed_inputs(wind_vector_x, wind_vector_y, lidar_samples)

        if recovery_x_error < 100.00 or self._flag_status == PilotFlags.RECOVER:
            self._flag_status = PilotFlags.RECOVER
            self.speed_ctrl.update_airspeed(
                distance=(recovery_x_error, recovery_y_error)
            )

        elif (
            # if object is diameter is larger that lidar_delivery_diameter check if collision avoidance is needed
            self.speed_ctrl.d_1_2 != None
            and self.speed_ctrl.d_1_2 > LIDAR_DELIVERY_DIAMETER
        ):
            if (
                self.speed_ctrl.distance[0] < VEHICLE_AVOID_THRESHOLD
                and self.speed_ctrl.distance[1] < LAT_AVOIDANCE_DISTANCE
            ):
                self._flag_status = PilotFlags.AVOID_COLLISION
                # change target distance to be away from tree collision boundary
                self.speed_ctrl.avoid_collision(lidar_samples)
                self.speed_ctrl.update_airspeed()
        elif self.speed_ctrl.d_1_2 != None:
            self._flag_status = PilotFlags.APPROACH_TARGET
            self.speed_ctrl.update_airspeed()

            v_x_sum = VEHICLE_AIRSPEED + wind_vector_x
            v_y_sum = self.speed_ctrl.v_y + wind_vector_y

            """Issue where packages were being dropped to trees on the corner of the lidar scans
            set lidar boundary to not drop if the object is located at the edges of vehicle bounds"""
            # TODO: adjust lateral speed with proportion to distance from the center of lidar scanner
            if abs(self.speed_ctrl.theta) < 10.0:
                # update the package controller to check for drop
                self._d_rel = (v_x_sum * PACKAGE_FALL_SEC, v_y_sum * PACKAGE_FALL_SEC)
                self.package_ctrl.update_target_params(
                    position=self.speed_ctrl._distance,
                    current_drop_pos=self._d_rel,
                    current_time=timestamp,
                )
            elif abs(self.speed_ctrl.theta) < 12.0:
                #increase the speed by 20% if target is on the edges of the lidar boundary
                self.speed_ctrl._v_y = self.speed_ctrl.v_y * 1.2
            elif abs(self.speed_ctrl.theta) > 12.0:
                #increase the speed by 20% if target is on the edges of the lidar boundary
                self.speed_ctrl._v_y = self.speed_ctrl.v_y * 1.5

    def return_data(self):
        return (self.speed_ctrl.v_y, self.package_ctrl.drop_status)