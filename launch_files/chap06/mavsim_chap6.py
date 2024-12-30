"""
mavsim_python
    - Chapter 6 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/5/2019 - RWB
        2/24/2020 - RWB
        1/5/2023 - David L. Christiansen
        7/13/2023 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
# use QuitListener for Linux or PC <- doesn't work on Mac
#from tools.quit_listener import QuitListener
import numpy as np
import parameters.simulation_parameters as SIM
from tools.signals import Signals
from models.quad_dynamics_control import QuadDynamicsControl
from models.wind_simulation import WindSimulation
from controllers.autopilot_fixedWing import Autopilot
#from controllers.autopilot_tecs import Autopilot
#from controllers.lqr_with_rate_damping import Autopilot
from viewers.view_manager import ViewManager
import time
from message_types.msg_sensors import MsgSensors

import pandas as pd

from copy import copy

#quitter = QuitListener()

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
quad = QuadDynamicsControl(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
viewers = ViewManager(data=True,
                      animation=True)

# autopilot commands
from message_types.msg_autopilot_fixedWing import MsgAutopilot
commands = MsgAutopilot()
Va_command = Signals(dc_offset=25.0,
                     amplitude=0.0,
                     start_time=2.0,
                     frequency=0.01)
altitude_command = Signals(dc_offset=100.0,
                           amplitude=0.0,
                           start_time=0.0,
                           frequency=0.05)
course_command = Signals(dc_offset=np.radians(0.0),
                         amplitude=np.radians(0.0),
                         start_time=5.0,
                         frequency=0.015)

# initialize the simulation time
sim_time = SIM.start_time
end_time = SIM.end_time


#creates the list to store the state through time
stateList = []

#creates the list to store the wrench through time
wrenchList = []

# main simulation loop
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
    current_wind = wind.update()  # get the new wind vector
    quad.update(delta, current_wind)  # propagate the MAV dynamics

    # ------- update viewers -------
    viewers.update(
        sim_time,
        true_state=quad.true_state,  # true states
        commanded_state=commanded_state,  # commanded states
        delta=delta, # inputs to MAV
        measurements=MsgSensors(),
        estimated_state=quad.true_state
    )

    #concatenates on the state
    stateList.append(quad.true_state)

    #gets the calculated wrench
    wrench = quad._forces_moments(delta=delta)
    #concatenates onto the wrench storage vector
    wrenchList.append(wrench)
    # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation

#writes the forces and moments on the aircraft to the output file

#gets the absolutePath of the trajectory follower folder
absolutePath = os.path.abspath("launch_files/trajectoryFollower/aircraftLineFollower/referenceData")

#changes the wrench list to a numpy array
wrenchList = np.array(wrenchList)[:,:,0].T

#writes the wrench List to a file in the trajectory follower path
dataFrame = pd.DataFrame(wrenchList)
dataFrame.to_csv(absolutePath + "/trueWrenches.csv")

#writes it to the low level controls file
dataFrame.to_csv('launch_files/lowLevelControls/aircraftControllers/wrenchReference.csv')

potato = 0

