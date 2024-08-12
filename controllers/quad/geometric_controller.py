#creates the geometric controller for the airplane

import numpy as np
from message_types.msg_trajectory import MsgTrajectory
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
import parameters.quad.anaconda_parameters as QUAD
from tools.differentiators import RotationDerivative
from tools.lie_group import vee_SO3, antisymmetric

from scipy.linalg import expm
from tools.rotations import quaternion_to_euler, quaternion_to_rotation, vee, hat
import parameters.quad.
