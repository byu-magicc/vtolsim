#this file will be used for debugging the forces moments functions, to make sure I have copied
#and moved things over correctly.

import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[4]))
import numpy as np
import parameters.quad.simulation_parameters as SIM
from models.quad.quad_dynamics import QuadDynamics
from controllers.quad.autopilot_fixedWing import Autopilot

from viewers.quad.view_manager import ViewManager
from message_types.quad.msg_autopilot_fixedWing import MsgAutopilotFixedWing
from tools.signals import Signals

from controllers.quad.low_level_control import LowLevelControl_simultaneousControl

import pandas as pd

#imports the class for the wrench calculation
from controllers.quad.forces_torques_derivatives import wrenchCalculation


#instantiates the quad

quad = QuadDynamics(ts=SIM.ts_simulation)

#instantiates the autopilot
autopilot = Autopilot(ts_control=SIM.ts_control)
#creates the view manager
viewers = ViewManager(animation=True, data=True)

#creates the low level controller
lowLevelController = LowLevelControl_simultaneousControl(ts=SIM.ts_simulation)

#creates the wrench calculator
wrenchCalculator = wrenchCalculation()

commands = MsgAutopilotFixedWing()

Va_command = Signals(dc_offset=20.0,
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

#creates the vector for the calculated forces and moments through all time steps
calculatedWrench = np.ndarray((5,0))

#creates the counter 
counter = 0

#creates the settle time to set how long it takes to settle
settleTime = 15.0

settleCount = int(settleTime/SIM.ts_simulation)


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

    #checks if we have settled.
    #case we have not yet settled and we need to use the autopilot temporarily
    if counter < settleCount:
        delta, commanded_state = autopilot.update(commands, estimated_state)
    else:
        delta = lowLevelController.update(f_d=f_d,
                                          omega_d=omega_d,
                                          state=quad.true_state,
                                          wind=wind)
        
    ###########################################################################################################L
    #does the forces comparisons

    #gets the actual forces moments from the quad, and compares them to the estimator's forces and moments
    forcesMomentsActual = quad._forces_moments(delta=delta)

    #gets the forces moments calculated from the 
    forcesMomentsCalculated, JacobianCalculated  = wrenchCalculator.forces_moments_derivatives(delta=delta, state=quad.true_state)

    #appends for each of them
    actualWrench = np.concatenate((actualWrench, forcesMomentsActual), axis=1)

    calculatedWrench = np.concatenate((calculatedWrench, forcesMomentsCalculated), axis=1)

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






    #increments the counter
    counter += 1

    #increments the sim time
    sim_time += SIM.ts_simulation


#gets the wrench error vector
wrenchError = lowLevelController.getWrenchError()


#writes them out
df_Actual = pd.DataFrame(actualWrench)
df_Actual.to_csv('/home/dben1182/Documents/vtolsim/launch_files/quad/lowLevelControlTests/fixedWingTests/verificationOutputs/actualWrench.csv', header=False, index=False)


df_Calculated = pd.DataFrame(calculatedWrench)
df_Calculated.to_csv('/home/dben1182/Documents/vtolsim/launch_files/quad/lowLevelControlTests/fixedWingTests/verificationOutputs/calculatedWrench.csv', header=False, index=False)


df_wrenchError = pd.DataFrame(wrenchError)
df_wrenchError.to_csv('/home/dben1182/Documents/vtolsim/launch_files/quad/lowLevelControlTests/fixedWingTests/verificationOutputs/wrenchError.csv', header=False, index=False)

