
from os import error
import numpy as np
import scipy.optimize as sco
from enum import Enum

from tools.rotations import quaternion_to_rotation, vee
from tools.wrap import wrap
import parameters.quad.anaconda_parameters as QUAD
import parameters.quad.optimal_pitch_parameters as OPT_PARAM



class OPTIMAL_PITCH_METHOD(Enum):
    Sampled = 1
    Optimizer = 2
    ZThrust = 3

class AERO_TYPE(Enum):
    SMALL_ANGLE = 1
    FLAT_PLATE_1 = 2
    FLAT_PLATE_2 = 3
    BLENDED_1 = 4
    BLENDED_2 = 5
    SMALL_ANGLE_CONTINUOUS = 6