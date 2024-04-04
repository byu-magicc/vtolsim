#this file implements the simulation file which will optimally allocate our tiltrotor's
#state in both multicopter and fixed wing mode. We will be using this in such a manner that 
#it minimizes the time and energy expended in multicopter mode. We're only making money when
#we're moving, and we're only moving quickly in fixed wing mode.

#our formatting and methodology will be the same as with the vtolsim_lqr.py file, which currently
#works correctly

#imports os and system
import os
import sys

#imports path so we can look outside our present folder
from pathlib import Path
sys.path.insert(0, os.fspath(Path(__file__).parents[1]))

#imports numpy
import numpy as np

#gets the simulation parameters
import parameters.simulation_parameters as SIM
#gets the spline parameters, which I presume are for spline path following
import parameters.spline_parameters as SPLP
#imports the msg convert functions
from message_types.msg_convert import *
#gets vtoldynamics
from models.vtol_dynamics import VtolDynamics
#imports the wind simulation
from models.wind_simulation import WindSimulation

###########################################################################
#imports the various controllers for this one
#gets lqr control
from controllers.lqr.lqr_control import LqrControl
#imports low level control
from controllers.low_level_control import LowLevelControl
#imports the rate controller
from controllers.rate_control import RateControl
#imports the attitude controller
from controllers.attitude_control import AttitudeControl

#imports the control allocation function
#TODO need to implement this
###########################################################################

#imports spline trajectories
from planners.trajectory.spline_trajectory import SplineTrajectory
#imports differential flatness, whatever that is
from planners.trajectory.differential_flatness import DifferentialFlatness
#imports msg delta, or control commands (aileron, elevator, rudder, and throttle)
from message_types.msg_delta import MsgDelta
#imports the ViewManager, which does all our animations and dataplotting
from viewers.view_manager import ViewManager

#imports the signals function
from tools.signals import Signals




#sets the wind vector
wind = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T
#gets the Vtol Dynamics
vtol = VtolDynamics(SIM.ts_simulation)
#uses the viewer from view manager, which one actually works
viewers = ViewManager(animation=True, data=True)
#initializes the wind simulation
wind = WindSimulation()


#Trim setup
#this section gets and sets up the trim




