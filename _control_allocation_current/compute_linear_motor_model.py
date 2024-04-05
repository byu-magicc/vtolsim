

#imports the system and os
import sys
import os

#gets the path so that we can see outside of this folder
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))



#gets the libraries for the rotations
from tools.rotations import quaternion_to_euler, quaternion_to_rotation

#imports the controls message
from message_types.msg_controls import MsgControls

#imports the Vtol Dynamics
from models.vtol_dynamics import VtolDynamics

#imports the trim
from tools.trim import compute_trim

#imports the convergence parameters
import parameters.convergence_parameters as VTOL_CONV

import numpy as np




#defines the compute rotor allocation submatrix