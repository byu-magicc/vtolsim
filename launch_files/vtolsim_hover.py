"""
edit history
    2/5/2019 - RWB
    11/20/2023 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
import pyqtgraph as pg
import parameters.simulation_parameters as SIM
import parameters.convergence_parameters as VTOL
from viewers.vtol_viewer import VtolViewer
from viewers.data_viewer import DataViewer
from models.vtol_dynamics import VtolDynamics
from models.wind import WindSimulation
from tools.signals import Signals
from message_types.msg_autopilot import MsgAutopilot
from controllers.hover_autopilot import HoverController


# initialize the visualization
plot_app = pg.QtWidgets.QApplication([])
vtol_view = VtolViewer(
    app=plot_app, dt=SIM.ts_simulation,
    plot_period=SIM.ts_plot_refresh)
data_view = DataViewer(
    app=plot_app,dt=SIM.ts_simulation, 
    plot_period=SIM.ts_plot_refresh,
    data_recording_period=SIM.ts_plot_record_data, 
    time_window_length=30)  

# initialize elements of the architecture
vtol = VtolDynamics(SIM.ts_simulation)
wind = WindSimulation()
ctrl = HoverController(SIM.ts_simulation)

# set initial state
vtol.set_state(np.array([
    [0.],  # [0]  north position
    [0.],  # [1]  east position
    [0.],  # [2]  down position
    [0.],   # [3]  velocity along body x-axis
    [0.],   # [4]  velocity along body y-axis
    [0.],   # [5]  velocity along body z-axis
    [1.],  # [6]  quaternion - scalar part
    [0.],  # [7]  quaternion - vector-x
    [0.],  # [8]  quaternion - vector-y
    [0.],  # [9]  quaternion - vector-z
    [0.], # [10]  roll rate
    [0.], # [11]  pitch rate
    [0.], # [12]  yaw rate
    [np.pi/2], # [13] pitch angle of right motor
    [np.pi/2],  # [14] pitch angle of left motor
]))

# initialize command signals
pn_command = Signals(amplitude=2., frequency=0.05)
pe_command = Signals(amplitude=2.0, frequency=0.05, start_time=10.)
h_command = Signals(amplitude = 5, frequency = 0.01, dc_offset=5)
psi_command = Signals(amplitude=3*np.pi/4, frequency=0.1, state_time=5)
commands = MsgAutopilot()

# initialize the simulation time
sim_time = SIM.start_time
# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # #-------autopilot commands-------------
    commands.pn = pn_command.square(sim_time)
    commands.pe = pe_command.square(sim_time)   
    commands.altitude = h_command.square(sim_time)
    commands.heading = psi_command.square(sim_time)

    #-------controller-------------
    estimated_state = vtol.state  # uses true states in the control
    delta, commanded_state = ctrl.update(commands, estimated_state)
    # -------physical system-------------
    current_wind = wind.update()
    vtol.update(delta, current_wind)
    # -------update viewer-------------
    vtol_view.update(vtol.state) 
    data_view.update(vtol.state,  # true states
                     estimated_state,  # estimated states
                     commanded_state,  # commanded states
                     delta)  # inputs to the vtol
    # -------increment time-------------
    sim_time += SIM.ts_simulation
