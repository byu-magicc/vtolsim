"""
vtolsim_attide
    - attitude control for VTOL
    - Update history:
        5/8/2019 - R.W. Beard
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
from controllers.attitude_control import AttitudeControl
from controllers.low_level_control import LowLevelControl

from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta

from tools.signals import Signals

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

# initialize elements of the architecture
wind = np.array([[0., 0., 0., 0., 0., 0.]]).T
vtol = VtolDynamics()
#initialize low level control
att_ctrl = AttitudeControl(SIM.ts_simulation)
low_ctrl = LowLevelControl(M=0.5, Va0=15.0, ts_control=SIM.ts_simulation)

phi_command = Signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.01)
theta_command = Signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.01)
psi_command = Signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.01)

# initialize the simulation time
sim_time = SIM.start_time
Ts = SIM.ts_simulation

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = vtol.sensors()  # get sensor measurements
    estimated_state = vtol.true_state  # estimated state is current state
    #-------controller-------------
    commanded_state = vtol.true_state  # commanded state is current state
    commanded_state.phi = phi_command.square(sim_time)
    #commanded_state.theta = theta_command.square(sim_time)
    #commanded_state.psi = psi_command.square(sim_time)
    # compute desired attitude
    att_cmd = np.array([
        [commanded_state.phi],
        [commanded_state.theta], 
        [commanded_state.psi], 
        ])
    # compute desired angular rate
    omega_d = att_ctrl.update(att_cmd, vtol.true_state)
    commanded_state.p = omega_d[0,0]
    commanded_state.q = omega_d[1,0]
    commanded_state.r = omega_d[2,0]

    Tz = .01*(-.1*np.clip(estimated_state.h, -10., 10.)) + .8394242
    Tz = np.clip(Tz, 0., 1.)
    print(Tz)
    delta = low_ctrl.update(omega_d, np.array([[0.0],[Tz]]), vtol.true_state)

    #-------update physical system-------------
    vtol.update(delta, wind)  # propagate the MAV dynamics
#-------update viewers-------------
    true_state = MsgState(vtol.true_state)  # convert old to new format
    vtol_view.update(true_state) 
    data_view.update(true_state,  # true states
                     true_state,  # estimated states
                     true_state,  # commanded states
                     MsgDelta(delta))  # inputs to the vtol     
    #-------increment time-------------
    sim_time += Ts

input("Press a key to exit")
