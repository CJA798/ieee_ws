from enum import Enum

class Areas(Enum):
    '''Enum class for the different areas of the map'''
    INITIAL_AREA = "INITIAL_AREA"
    BIG_PACKAGE_WALL = "BIG_PACKAGE_WALL"
    PUSH_BIG_PACKAGES = "PUSH_BIG_PACKAGES"
    DROP_OFF = "DROP_OFF"
    FUEL_TANK = "FUEL_TANK"
    CRATER = "CRATER"
    BUTTON = "BUTTON"
    RE_SCAN = "RE_SCAN"
