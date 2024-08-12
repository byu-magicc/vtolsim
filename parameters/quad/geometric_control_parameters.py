

from os import error
import numpy as np
import scipy.optimize as sco
from enum import Enum

from tools.rotations import quaternion_to_rotation, vee
from tools.wrap import wrap
import parameters.quad.anaconda_parameters as QUAD
import parameters.quad.optimal