from enum import IntFlag


class PackageFlags(IntFlag):
    DONT_DROP_PACKAGE = 0
    DROP_PACKAGE = 1