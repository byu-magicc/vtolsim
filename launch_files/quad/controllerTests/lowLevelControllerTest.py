#this file's purpose is to perform a test of the low level controller 

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
from controllers.quad.low_level_control import LowLevelControl_simultaneousControl
from message_types.quad.msg_delta import MsgDelta
from viewers.quad.view_manager import ViewManager




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

    #gets the delta output for the system
    delta = lowControl.update(f_d=F_desired,
                              omega_d=omega_desired,
                              state=quad.true_state,
                              wind=wind)
    
    quad.update(delta=delta, wind=wind)

    #updates the viewer
    viewers.update(sim_time=sim_time,
                   true_state=quad.true_state,
                   estimated_state=quad.true_state,
                   commanded_state=quad.true_state,
                   delta=delta,
                   measurements=None)
    
    if printerCounter % 100 == 0:
        a = 0

    printerCounter += 1

    #increments the time
    sim_time += SIM.ts_simulation