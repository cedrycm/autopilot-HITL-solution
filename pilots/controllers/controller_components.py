from __future__ import annotations
from enum import IntFlag

# for autopilot calcs
from operator import itemgetter
from math import sqrt, cos, sin, tan, atan, radians, degrees
import struct
import random
import numpy as np
import itertools


AIRSPEED_X = 30.0

MAX_AIRSPEED = 30.0

MAX_LIDAR_DISTANCE = 255

MIN_LIDAR_ANGLE = -15.0

MAX_LIDAR_ANGLE = 15.0

WORLD_WIDTH = 50.0

WORLD_LENGTH = 2000.0
# How long it takes for the package to "fall" and hit the ground after being released.
PACKAGE_FALL_SEC = 0.5

DELIVERY_SITE_RADIUS = 5.0

MAX_AVOID_ANGLE = 15.0

VEHICLE_AVOID_THRESHOLD = 30


class PackageFlags(IntFlag):
    DONT_DROP_PACKAGE = 0
    DROP_PACKAGE = 1


# ------=Utility Functions-------------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
def euclid_values(idx_1, idx_2, data: list):
    p_1 = data[idx_1]
    p_2 = data[idx_2]
    d_1 = p_1
    d_2 = p_2

    theta_1 = (-1) * (idx_1) + 15
    theta_2 = (-1) * (idx_2) + 15

    x_1 = d_1 * cos(radians(theta_1))
    y_1 = d_1 * sin(radians(theta_1))

    x_2 = d_2 * cos(radians(theta_2))
    y_2 = d_2 * sin(radians(theta_2))
    d_y = abs(y_1 - y_2)

    # euclidian distance between points
    d_1_2 = sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2)
    # get midpoint
    m_x = (x_1 + x_2) / 2
    m_y = (y_1 + y_2) / 2
    distance = sqrt(m_x ** 2 + m_y ** 2)
    theta = degrees(atan(m_y / m_x))
    return (d_1_2, distance, theta, theta_1, theta_2)


def group_adjacent(data):
    if np.count_nonzero(data) > 0:
        nonzero = np.nonzero(data)
        samples = nonzero[0].tolist()

        for _, value in itertools.groupby(
            enumerate(samples), lambda i_x: i_x[0] - i_x[1]
        ):
            consecutive_groups = map(itemgetter(1), value)
            consecutive_groups = list(consecutive_groups)
            if len(consecutive_groups) == 1:
                yield consecutive_groups[0]
            else:
                # group even further by distance
                subgroup = subgrouper(consecutive_groups, data)
                # subgroup = map(itemgetter(1), subgroup)
                # subgroup = list(subgroup)
                yield subgroup
    else:
        # if all non-zero, return front angle index
        return None
    # return np.split(test_array, np.where(np.diff(test_array) != 1)[0] + 1)


def subgrouper(iterable, data):
    for group in iterable:
        prev = None
        group = []
        for item in iterable:
            try:
                val = abs(data[item] - data[prev])
            except:
                val = 0
            if not prev or val <= 6.0 or val == 0:
                group.append(item)
            else:
                yield group
                group = [item]
            prev = item
            if group:
                yield group


# ------CONTROLLER COMPONENTS---------------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
class Detector:
    __slots__ = ["_lidar_matrix", "_d_1_2", "_distance", "_theta", "_theta1", "_theta2"]

    def __init__(self):
        self._lidar_matrix = np.zeros(shape=(5, 31), dtype=int)
        self._distance = (0.0, 0.0)
        self._theta = 0.0
        self._theta1 = None
        self._theta2 = None
        self._d_1_2 = 0.0

    @property
    def d_1_2(self):
        return self._d_1_2

    @property
    def distance(self):
        return self._distance

    @distance.setter
    def distance(self, value):
        self._distance = (
            value * cos(radians(self._theta)),
            value * sin(radians(self._theta)),
        )

    @property
    def theta(self):
        return self._theta

    @theta.setter
    def theta(self, value):
        self._theta = value

    @property
    def theta1(self):
        return self._theta1

    @property
    def theta2(self):
        return self._theta1

    @property
    def lidar_samples(self):
        return self._lidar_matrix

    @lidar_samples.setter
    def lidar_samples(self, value: list):
        if len(value) == 31:
            self._lidar_matrix = np.delete(self._lidar_matrix, 0, 0)
            self._lidar_matrix = np.append(self._lidar_matrix, [value], axis=0)
            self.interpret_lidar()

    def interpret_lidar(self):
        median_samples = np.median(self._lidar_matrix, axis=0).tolist()
        (d_1_2, distance, theta, theta1, theta2) = self.get_closest_object(
            median_samples
        )
        self._d_1_2 = d_1_2
        self._theta = theta
        self._theta1 = theta1
        self._theta2 = theta2
        self.distance = distance

    def get_closest_object(self, median_samples):
        # Find possible delivery by isolating lidar points of contact
        distance_nearest = MAX_LIDAR_DISTANCE
        theta_nearest = MAX_LIDAR_ANGLE
        d_1_2_nearest = None
        theta1_nearest = None
        theta2_nearest = None
        try:
            for _, idx_lists in enumerate(group_adjacent(median_samples)):

                if not isinstance(idx_lists, int):
                    index_set = list(idx_lists)[0]
                    # clean_set = detect_outlier(index_set, self.lidar_samples)
                    location = euclid_values(
                        index_set[0], index_set[-1], median_samples
                    )
                    d_1_2 = location[0]
                    distance = location[1]
                    theta = location[2]
                    theta1 = location[3]
                    theta2 = location[4]

                    if distance < distance_nearest:

                        d_1_2_nearest = d_1_2
                        distance_nearest = distance
                        theta_nearest = theta
                        theta1_nearest = theta1
                        theta2_nearest = theta2
                else:
                    # for single point
                    d_1_2 = 0.0
                    idx = int(idx_lists)
                    distance = median_samples[idx]
                    theta = (-1) * (idx) + 15

                    # if single point is the closest, approach, but do not drop
                    if distance < distance_nearest:
                        # assign to 'nearest' values
                        d_1_2_nearest = d_1_2
                        distance_nearest = distance
                        theta_nearest = theta
        except Exception as e:
            # catch when lidar samples are all 0 values
            return (0, 0, 0, 0, 0)

        return (
            d_1_2_nearest,
            distance_nearest,
            theta_nearest,
            theta1_nearest,
            theta2_nearest,
        )


class SpeedController(Detector):
    __slots__ = [
        "_v_y",
        "_v_x",
        "_wind_vector_x",
        "_wind_vector_y",
    ]

    def __init__(self, v_x=AIRSPEED_X):
        self._v_y = 0.0
        self._v_x = v_x
        self._wind_vector_x = 0.0
        self._wind_vector_y = 0.0
        super().__init__()

    @property
    def v_y(self):
        return self._v_y

    def speed_inputs(self, wind_vector_x, wind_vector_y, lidar_samples):
        self.lidar_samples = lidar_samples
        self._wind_vector_x = wind_vector_y
        self._wind_vector_y = wind_vector_y

    def update_airspeed(self, distance=None):
        v_x_sum = self._v_x + self._wind_vector_x
        # update lateral airspeed using kinematics of lateral component
        if distance is None:
            theta = self.theta
        else:
            if distance[0] != 0:
                theta = degrees(atan(distance[1] / distance[0]))
            else:
                theta = 0
        #update airspeed        
        self._v_y = (v_x_sum * atan(radians(theta))) - self._wind_vector_y      

    def avoid_collision(self, lidar_samples):
        theta_last = MAX_LIDAR_ANGLE
        distance_last = VEHICLE_AVOID_THRESHOLD
        
        for target_idx, distance in enumerate(lidar_samples):
            theta = (-1) * (target_idx) + 15
            d_x = distance * cos(radians(theta_last))

            #1 degree extra buffer for no collision direction
            if not (self.theta1 - 1  < theta < self.theta2 + 1) or (
                self.theta2 - 1 < theta < self.theta1 + 1
            ):
                if (d_x > VEHICLE_AVOID_THRESHOLD or distance == 0) and abs(
                    theta 
                ) < abs(theta_last):
                    distance_last = distance
                    theta_last = theta

        self.theta = theta_last


class PackageController:
    __slots__ = ["_drop_flag", "_drop_timestamp", "_target_center"]

    def __init__(self):
        self._drop_flag = PackageFlags.DONT_DROP_PACKAGE
        self._drop_timestamp = 0
        self._target_center = None

    @property
    def drop_status(self):
        if self._drop_flag == PackageFlags.DONT_DROP_PACKAGE:
            return 0
        else:
            return 1

    def update_target_params(self, position, current_drop_pos, current_time):
        # update potential target position and relative velocity to ground
        self._target_center = position
        self.check_target(current_drop_pos, current_time)
        return None

    def check_target(self, current_drop_pos, current_time):
        # find if target is within drop buffer zone
        time_elapsed = current_time - self._drop_timestamp
        # wait time elapsed after a drop to check for new target
        if 0 < time_elapsed <= 500:
            self._drop_flag = PackageFlags.DONT_DROP_PACKAGE
        else:
            # reset drop timestamp once no longer in area of previous drop
            self._drop_timestamp = current_time
            if self.contains(current_drop_pos):
                self._drop_flag = PackageFlags.DROP_PACKAGE
                self._drop_timestamp = current_time

    def contains(self, position):
        delta_x = abs(self._target_center[0] - position[0])
        delta_y = abs(self._target_center[1] - position[1])

        # give a smaller buffer radius to account for wind fluctuation
        return delta_x ** 2 + delta_y ** 2 < (DELIVERY_SITE_RADIUS - 1.0) ** 2
     