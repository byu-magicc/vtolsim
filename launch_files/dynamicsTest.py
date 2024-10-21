import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np
import parameters.simulation_parameters as SIM
from tools.signals import Signals
from models.quad_dynamics import QuadDynamics
from models.wind_simulation import WindSimulation
from message_types.msg_sensors import MsgSensors

from viewers.view_manager import ViewManager
import time

from controllers.trimValues import trimDelta

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = QuadDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True,
                      data=True)

# autopilot commands
from message_types.msg_autopilot_fixedWing import MsgAutopilotFixedWing
commands = MsgAutopilotFixedWing()
Va_command = Signals(dc_offset=25.0,
                     amplitude=3.0,
                     start_time=2.0,
                     start_frequency=0.01)
altitude_command = Signals(dc_offset=100.0,
                           amplitude=20.0,
                           start_time=0.0,
                           start_frequency=0.02)
course_command = Signals(dc_offset=np.radians(180),
                         amplitude=np.radians(45),
                         start_time=5.0,
                         start_frequency=0.015)


sim_time = SIM.start_time
end_time = SIM.end_time

# main simulation loop
print("Press 'Esc' to exit...")

while sim_time < end_time:

    # -------autopilot commands-------------
    commands.airspeed_command = Va_command.square(sim_time)
    commands.course_command = course_command.square(sim_time)
    commands.altitude_command = altitude_command.square(sim_time)

    # -------autopilot-------------
    estimated_state = mav.true_state  # uses true states in the control
    delta = trimDelta 

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    # ------- update viewers -------
    viewers.update(
        sim_time,
        true_state=mav.true_state,  # true states
        commanded_state=mav.true_state,  # commanded states
        estimated_state=mav.true_state,
        measurements=MsgSensors(),
        delta=delta, # inputs to MAV
    )
       
    # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation
    time.sleep(0.002) # slow down the simulation for visualization

viewers.close(dataplot_name="ch6_data_plot")