#This file implements a trajectory following aircraft. Let's see if it really works

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
import numpy as np
import time
import parameters.simulation_parameters as SIM

from viewers.view_manager import ViewManager
from models.quad_dynamics import QuadDynamics
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from controllers.low_level_control import LowLevelControl_simultaneousControl
from controllers.rate_control import RateControl
from trajectory.pitch_free_trajectory_tracker import PitchFreeTrajectoryTracker
from trajectory.pitch_control import PitchControl
from trajectory.attitude_control import AttitudeControl
from tools.rotations import quaternion_to_euler, rotation_to_quaternion, rotation_to_euler, quaternion_to_rotation
import pandas as pd


from trajectory.plannedTrajectories.lineTrajectories import lineFlight

from tools.performanceMeasures import performanceMeasures



#creates the main function
def main()



#calls the main function
main()