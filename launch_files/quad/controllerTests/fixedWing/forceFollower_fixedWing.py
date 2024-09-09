#this file implements a force and torque following algorithm for the airplane in 
#fixed wing mode

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[4]))

import pandas as pd

import numpy as np
import parameters.quad.simulation_parameters as SIM

#imports the quad dynamics to create the plane model
from models.quad.quad_dynamics import QuadDynamics
#imports the low level controller for the Quad,
#which minimizes the mean squared error
from tools.signals import Signals

from controllers.quad.low_level_control import LowLevelControl_SurfacesShortened
from message_types.quad.msg_delta import MsgDelta
from viewers.quad.view_manager import ViewManager
from message_types.quad.msg_state import MsgState
import pandas as pd

from models.quad.trimValues import trimDelta
import models.quad.trimValues as TRIM