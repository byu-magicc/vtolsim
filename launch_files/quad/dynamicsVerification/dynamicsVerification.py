#This file implements a simple controller to be enabled to compare with MAVSIM, so I can find out what I
#am doing wrong with my dynamics and my function and system verification.

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))
# use QuitListener for Linux or PC <- doesn't work on Mac
#from tools.quit_listener import QuitListener
import numpy as np
import parameters.quad.simulation_parameters as SIM
from tools.signals import Signals
from models.quad.quad_dynamics import QuadDynamics
from controllers.quad.autopilot_fixedWing import Autopilot
#from controllers.autopilot_tecs import Autopilot
#from controllers.lqr_with_rate_damping import Autopilot
from viewers.quad.view_manager import ViewManager
import time
from message_types.quad.msg_autopilot_fixedWing import MsgAutopilotFixedWing

import csv


import pandas as pd


#instantiates the quad

quad = QuadDynamics(ts=SIM.ts_simulation)

#instantiates the autopilot
autopilot = Autopilot(ts_control=SIM.ts_control)
#creates the view manager
viewers = ViewManager(animation=True, data=True)



commands = MsgAutopilotFixedWing()

Va_command = Signals(dc_offset=25.0,
                     amplitude=0.0,
                     start_time=2.0,
                     start_frequency=0.01)
altitude_command = Signals(dc_offset=100.0,
                           amplitude=0.0,
                           start_time=0.0,
                           start_frequency=0.05)
course_command = Signals(dc_offset=np.radians(0.0),
                         amplitude=np.radians(0.0),
                         start_time=5.0,
                         start_frequency=0.015)


#creates the vector for the actual forces and moments through all time steps
actualWrench = np.ndarray((6,0))


#creates the vector for the actual forces and moments through all time steps
actualWrench = np.ndarray((6,0))

#creates the deltas
deltaArray = np.ndarray((8,0))

sim_time = SIM.start_time
end_time = SIM.end_time

print("Press 'Esc' to exit...")
while sim_time < end_time:

    # -------autopilot commands-------------
    commands.airspeed_command = Va_command.square(sim_time)
    commands.course_command = course_command.square(sim_time)
    commands.altitude_command = altitude_command.square(sim_time)

    estimated_state = quad.true_state

    measurements = quad.sensors()

    #sets the f_d
    f_d = np.array([[0.0],[0.0]])

    #sets the omega
    omega_d = np.array([[0.0],[0.0],[0.0]])

    wind = np.array([[0.0],[0.0],[0.0]])

    delta, commanded_state = autopilot.update(commands, estimated_state)

        
    ###########################################################################################################L
    #does the forces comparisons

    #gets the actual forces moments from the quad, and compares them to the estimator's forces and moments
    forcesMomentsActual = quad._forces_moments(delta=delta)

    #gets the forces moments calculated from the 

    #appends for each of them
    actualWrench = np.concatenate((actualWrench, forcesMomentsActual), axis=1)

    #gets the deltas
    deltaArrayTemp = delta.to_array()

    deltaArray = np.concatenate((deltaArray, deltaArrayTemp), axis=1)
    ###########################################################################################################L


    # -------physical system-------------
    current_wind = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]) # get the new wind vector
    quad.update(delta, current_wind)  # propagate the MAV dynamics   

    # ------- update viewers -------
    viewers.update(
        sim_time,
        true_state=quad.true_state,  # true states
        commanded_state=quad.true_state,  # commanded states
        estimated_state=estimated_state,
        measurements=measurements,
        delta=delta, # inputs to MAV
    )


    #increments the sim time
    sim_time += SIM.ts_simulation



df1 = pd.DataFrame(deltaArray)
df1.to_csv("/home/dben1182/Documents/vtolsim/launch_files/quad/dynamicsVerification/vtolsimDeltas.csv", header=False, index=False)

df2 = pd.DataFrame(actualWrench)
df2.to_csv("/home/dben1182/Documents/vtolsim/launch_files/quad/dynamicsVerification/vtolsimWrench.csv", header=False, index=False)

