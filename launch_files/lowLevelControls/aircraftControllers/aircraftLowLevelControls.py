#this file launches and runs the low level controller for just the airplane controls
#as in the elevator, aileron, rudder and forward throttle contorls

import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))
import numpy as np
import parameters.simulation_parameters as SIM
from models.quad_dynamics import QuadDynamics
from controllers.autopilot_fixedWing import Autopilot

from viewers.view_manager import ViewManager
from message_types.msg_autopilot_fixedWing import MsgAutopilot
from tools.signals import Signals

#gets the wrench calculation
from controllers.forces_torques_derivatives import wrenchCalculation

#instantiates the quad
quad = QuadDynamics(ts=SIM.ts_simulation)

#creates the view maneger