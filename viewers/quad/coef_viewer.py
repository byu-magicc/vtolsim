#this file implements getting the viewer to view the real time changing estimates
#of the coefficients of aerodynamics

import numpy as np
from viewers.quad.plotter import Plotter
from tools.wrap import wrap
from tools.rotations import rotation_to_euler
from message_types.quad.msg_state import MsgState
from message_types.quad.msg_delta import MsgDelta