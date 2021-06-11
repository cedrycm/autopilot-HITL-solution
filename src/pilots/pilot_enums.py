from enum import IntFlag


class PackageFlags(IntFlag):
    DONT_DROP_PACKAGE = 0
    DROP_PACKAGE = 1


class PilotFlags(IntFlag):
    APPROACH_TARGET = 0
    AVOID_COLLISION = 1
    DELIVER_PACKAGE = 2
    RECOVER = 3