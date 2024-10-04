#this file implements a verificaiton function for the forces and torques and
#ensures that we are obtaining them appropriately and getting the right answer


#this file will do a normal autopilot thing, and then go and see if the actual forces
#match up with my own projected forces.


#creates launch file for a vtolsim

import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))
import numpy as np
import parameters.quad.simulation_parameters as SIM
from models.quad.quad_dynamics import QuadDynamics
from controllers.quad.autopilot_fixedWing import Autopilot

from viewers.quad.view_manager import ViewManager
from message_types.quad.msg_autopilot_fixedWing import MsgAutopilotFixedWing
from tools.signals import Signals
from message_types.quad.msg_sensors import MsgSensors

from models.quad.trimValues import trimDelta

#imports the forces torques function 
from controllers.quad.forces_torques_derivatives import wrenchCalculation

import pandas as pd


#instantiates the wrench calculation class
wrenchCalculator = wrenchCalculation()

#instantiates the quad
quad = QuadDynamics(ts=SIM.ts_simulation)
#instantiates the autopilot
autopilot = Autopilot(ts_control=SIM.ts_simulation)
viewers = ViewManager(animation=True,
                      data=True)


commands = MsgAutopilotFixedWing()

Va_command = Signals(dc_offset=25.0,
                     amplitude=5.0,
                     start_time=2.0,
                     start_frequency=0.05)
altitude_command = Signals(dc_offset=100.0,
                           amplitude=10.0,
                           start_time=0.0,
                           start_frequency=0.05)
course_command = Signals(dc_offset=np.radians(0.0),
                         amplitude=np.radians(10.0),
                         start_time=5.0,
                         start_frequency=0.015)


sim_time = SIM.start_time
end_time = SIM.end_time


#creates the vector for the actual forces and moments
actualWrench = np.ndarray((6,0))

#creates the vector for the projected forces and moments
calculatedWrench = np.ndarray((5,0))

counter = 0

print("Press 'Esc' to exit...")
while sim_time < end_time:

    # -------autopilot commands-------------
    commands.airspeed_command = Va_command.square(sim_time)
    commands.course_command = course_command.square(sim_time)
    commands.altitude_command = altitude_command.square(sim_time)

    # -------autopilot-------------
    estimated_state = quad.true_state  # uses true states in the control
    delta, commanded_state = autopilot.update(commands, estimated_state)



    # -------physical system-------------
    current_wind = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]) # get the new wind vector
    quad.update(delta, current_wind)  # propagate the MAV dynamics



    #saves the current forces and torques
    actualWrenchTemp = quad._forces_moments(delta=delta)
    #saves the calculated wrench temp
    calculatedWrenchTemp = wrenchCalculator.forces_moments_derivatives(delta=delta, state=quad.true_state)

    #appends them
    actualWrench = np.concatenate((actualWrench, actualWrenchTemp), axis=1)
    calculatedWrench = np.concatenate((calculatedWrench, calculatedWrenchTemp), axis=1)



    # ------- update viewers -------
    viewers.update(
        sim_time,
        true_state=quad.true_state,  # true states
        commanded_state=commanded_state,  # commanded states
        delta=delta, # inputs to MAV
        estimated_state=quad.true_state,
        measurements = MsgSensors()
    )

    counter += 1
       
    # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation

print(counter)

actualWrench_df = pd.DataFrame(actualWrench)
actualWrench_df.to_csv('//home//dben1182//Documents//vtolsim//launch_files//quad//functionVerification//actualWrench.csv', header=False, index=False)

calculatedWrench_df = pd.DataFrame(calculatedWrench)
calculatedWrench_df.to_csv('//home//dben1182//Documents//vtolsim//launch_files//quad//functionVerification//calculatedWrench.csv', header=False, index=False)