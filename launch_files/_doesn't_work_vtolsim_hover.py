"""
vtolsim_hover
    - Hover control for VTOL
        2/5/2019 - RWB
        2/1/2024 - RWB
"""
#/usr/bin/python3
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
import pyqtgraph as pg
import parameters.simulation_parameters as SIM

from viewers.vtol_viewer import VtolViewer
from viewers.data_viewer import DataViewer
from models.vtol_dynamics import VtolDynamics
from controllers.hover_controller.hover_autopilot import HoverController
from message_types.msg_controls import MsgControls
from message_types.msg_autopilot import MsgAutopilot
from tools.signals import Signals

from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta

# initialize viewers
plot_app = pg.QtWidgets.QApplication([])
vtol_view = VtolViewer(
    app=plot_app, dt=SIM.ts_simulation,
    plot_period=SIM.ts_plot_refresh)
data_view = DataViewer(
    app=plot_app,dt=SIM.ts_simulation, 
    plot_period=SIM.ts_plot_refresh,
    data_recording_period=SIM.ts_plot_record_data, 
    time_window_length=30) 

delta = MsgControls()

# initialize elements of the architecture
wind = np.array([[0., 0., 0., 0., 0., 0.]]).T
vtol = VtolDynamics()
ctrl = HoverController(SIM.ts_simulation)

# autopilot commands
commands = MsgAutopilot()
# Va_command = signals(dc_offset=25.0, amplitude=3.0, start_time=2.0, frequency = 0.01)
# h_command = signals(dc_offset=100.0, amplitude=10.0, start_time=0.0, frequency = 0.02)
# chi_command = signals(dc_offset=np.radians(180), amplitude=np.radians(45), start_time=5.0, frequency = 0.015)

h_command = Signals(dc_offset=100, amplitude=5, start_time=5.0, frequency = 0.0025)
pn_command = Signals(dc_offset=0, amplitude=5, start_time=10.0, frequency = 0.0025)
pe_command = Signals(dc_offset=0, amplitude=5, start_time=0.0, frequency = 0.0025)

# Va_star = 0.0
# gamma_star = 0.0
# # state_trim, delta_trim = compute_trim(vtol, Va_star, gamma_star)
# state_trim = np.array([[-1.04239742e-12],  [3.06873064e-12], [-1.00000000e+02], [-5.94131919e-07],
#    [0.00000000e+00],  [1.86190987e-06],  [9.99985895e-01],  [0.00000000e+00],
#    [5.31122783e-03],  [0.00000000e+00],  [0.00000000e+00],  [0.00000000e+00],
#    [0.00000000e+00],  [1.51897067e+00],  [1.59062520e+00]])

# delta_trim = np.array([[-0.05454118],  [0.06484209],  [0.78348849],  [0.64704488],  [0.64844094],  [1.51897067],
#    [1.5906252]])

# vtol._state = state_trim
# vtol._update_true_state()
# delta.elevon_right = delta_trim.item(0)
# delta.elevon_left = delta_trim.item(1)
# delta.throttle_rear = delta_trim.item(2)
# delta.throttle_right = delta_trim.item(3)
# delta.throttle_left = delta_trim.item(4)
# delta.servo_right = delta_trim.item(5)
# delta.servo_left = delta_trim.item(6)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------autopilot commands-------------
    commands.roll_command = 0.0#phi_command.square(sim_time)
    commands.pitch_command = 0.0#theta_command.square(sim_time)
    commands.yaw_command = 0.0#psi_command.square(sim_time)
    commands.altitude_command = h_command.square(sim_time)
    commands.pn_command = pn_command.square(sim_time)
    commands.pe_command = pe_command.square(sim_time)
    #-------controller-------------
    estimated_state = vtol.true_state  # uses true states in the control
    delta, commanded_state = ctrl.update(commands, estimated_state)
    #-------physical system-------------
    vtol.update(delta, wind)  # propagate the vtol dynamics

    #-------update viewer-------------
    true_state = MsgState(vtol.true_state)  # convert old to new format
    vtol_view.update(true_state) 
    data_view.update(true_state,  # true states
                     true_state,  # estimated states
                     true_state,  # commanded states
                     MsgDelta(delta))  # inputs to the vtol     
    #-------increment time-------------
    sim_time += SIM.ts_simulation
input("Press a key to exit")
