#this file just does the standard fixed wing piloting test portion.

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))

import numpy as np
import parameters.quad.simulation_parameters as SIM

#imports the quad dynamics to create the plane model
from models.quad.quad_dynamics import QuadDynamics
#imports the low level controller for the Quad,
#which minimizes the mean squared error
from tools.signals import Signals

from controllers.quad.lqr_fixedWing import Autopilot
from message_types.quad.msg_delta import MsgDelta
from viewers.quad.view_manager import ViewManager
from message_types.quad.msg_state import MsgState
import pandas as pd

from models.quad.trimValues import trimDelta
import models.quad.trimValues as TRIM

#creates  the wind#creates the wind
wind = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T

#instantiates the quadplane dynamics
quad = QuadDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)


#sets the initial states for altitude and airspeed
initialState = MsgState()
initialState.vel[0][0] = 25.0
initialState.pos[2][0] = -100.0

#sets the initial state for the plane
quad.setInitialConditions(initialTrueState=initialState)


#creates the airspeed, altitude, and course commands
from message_types.quad.msg_autopilot_fixedWing import MsgAutopilotFixedWing
commands = MsgAutopilotFixedWing()
Va_command = Signals(dc_offset=25.0,
                     amplitude=5.0,
                     start_time=2.0,
                     start_frequency=0.1)
altitude_command = Signals(dc_offset=100.0,
                           amplitude=0.0,
                           start_time=0.0,
                           start_frequency=0.05)
course_command = Signals(dc_offset=np.radians(0.0),
                         amplitude=np.radians(0.0),
                         start_time=5.0,
                         start_frequency=0.015)



#instantiates the controller
controller = Autopilot(ts_control=SIM.ts_control)

sim_time = SIM.start_time

counter = 0

while sim_time < SIM.end_time:

    #gets the commands
    commands.airspeed_command = Va_command.square(sim_time)
    commands.course_command = course_command.square(sim_time)
    commands.altitude_command = altitude_command.square(sim_time)

    measurements = quad.sensors()
    estimatedState = quad.true_state

    delta, commanded_state = controller.update(cmd=commands, state=quad.true_state)

    #sets the delta to trim
    #delta = trimDelta

    current_wind = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]) # get the new wind vector
    wrench = quad.update(delta=delta, wind=current_wind)

    
    if counter % 100 == 0:
        Fx = wrench[0][0]
        Fy = wrench[1][0]
        Fz = wrench[2][0]
        a=0


    viewers.update(sim_time=sim_time,
                   true_state=quad.true_state,
                   estimated_state=quad.true_state,
                   commanded_state=commanded_state,
                   delta=delta,
                   measurements=None)
    
    counter += 1
    
    sim_time += SIM.ts_simulation

    