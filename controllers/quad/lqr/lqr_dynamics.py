


import numpy as np

from tools.quaternions import q_circleTimes, q_conj, q_rotate, q_boxPlus, q_boxMinus, state_boxMinus, state_boxPlus
from tools.rotations import hat
from message_types.msg_controls import MsgControls
import parameters.control_allocation_parameters as CP


#creates the epsilon factor
epsilon = 0.001
g = QUAD