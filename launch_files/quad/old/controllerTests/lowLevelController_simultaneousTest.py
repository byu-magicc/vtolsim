#this file's purpose is to perform a test of the low level controller 

#for simultaneous control

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
from controllers.quad.old.low_level_control import LowLevelControl_simultaneousControl
from message_types.quad.msg_delta import MsgDelta
from viewers.quad.view_manager import ViewManager

import pandas as pd

import time




#creates the wind
wind = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T

#instantiates the quadplane dynamics
quad = QuadDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)


#instantiates the low level controller
lowControl = LowLevelControl_simultaneousControl(M=0.5, Va0=2.0, ts=SIM.ts_simulation)


#sets the simulation time
sim_time = SIM.start_time

printerCounter = 0

#creates an array to store the deltas for the data analysis
deltaOutputArray = np.ndarray((8,0))

#creates array to store the completion times of the 
completion_times = []

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:

    #gets the measurements
    measurements = quad.sensors()
    estimatedState = quad._state

    #sets the force desired to zero
    F_desired = np.array([[0.0], [0.0], [0.0]])

    #sets the p, q, r desired
    omega_desired = np.array([[0.0], [0.0], [0.0]])

    #gets the start time
    control_start_time = time.time()
    #gets the delta output for the system
    delta = lowControl.update(f_d=F_desired,
                              omega_d=omega_desired,
                              state=quad.true_state,
                              quad=quad,
                              wind=wind)
    #gets the control end time
    control_end_time = time.time()

    deltaArray = np.array([[delta.elevator],
                           [delta.aileron],
                           [delta.rudder],
                           [delta.forwardThrottle],
                           [delta.verticalThrottle_1],
                           [delta.verticalThrottle_2],
                           [delta.verticalThrottle_3],
                           [delta.verticalThrottle_4]])

    deltaOutputArray = np.concatenate((deltaOutputArray, deltaArray), axis=1)
    
    quad.update(delta=delta, wind=wind)

    #updates the viewer
    viewers.update(sim_time=sim_time,
                   true_state=quad.true_state,
                   estimated_state=quad.true_state,
                   commanded_state=quad.true_state,
                   delta=delta,
                   measurements=None)
    

    #appends on the control start and end times
    completion_times.append(control_end_time - control_start_time)

    printerCounter += 1

    #increments the time
    sim_time += SIM.ts_simulation


controlDataFrame = pd.DataFrame(completion_times)
controlDataFrame.to_csv('//home//dben1182//Documents//vtolsim//launch_files//quad//controllerTests//vtolSim_control_alloc_times.csv', header=False, index=False)



#writes the delta output array to a csv
dataFrame = pd.DataFrame(deltaOutputArray)

dataFrame.to_csv("//home//dben1182//Documents//vtolsim//launch_files//quad//controllerTests//SimultaneousTest.csv", index=False, header=False)


