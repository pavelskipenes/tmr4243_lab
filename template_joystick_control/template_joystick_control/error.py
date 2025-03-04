from enum import Enum


class Error(str, Enum):
    POSITION_INVALID_DIMENSION = "the dimensions of position is not 3x1"
    POSITION_MISSING = "the position is not set"
