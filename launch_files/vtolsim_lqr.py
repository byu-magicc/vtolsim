"""
vtolsim_lqr
    - simultion showing trajectory following using full state LQR
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
import parameters.spline_parameters as SPLP
from message_types.msg_controls import MsgControls
from message_types.msg_convert import *
from models.vtol_dynamics import VtolDynamics
from controllers.lqr.lqr_control import LqrControl
from controllers.low_level_control import LowLevelControl
from planners.trajectory.spline_trajectory import SplineTrajectory
from planners.trajectory.differential_flatness import DifferentialFlatness
from viewers.vtol_viewer import VtolViewer
from viewers.data_viewer import DataViewer

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

# initialize elements of the architecture
wind = np.array([[0., 0., 0., 0., 0., 0.]]).T
vtol = VtolDynamics()

# initialize trajectory
# df_traj = DifferentialFlatness(XYZSinusoid(150., 150., 75., 600., 300., 600., -np.pi/2, np.pi, 0.*np.pi/2))
# df_traj = DifferentialFlatness(HANDTraj())
df_traj = DifferentialFlatness(SplineTrajectory(SPLP.pts, SPLP.vels)) # points, max velocity
step =  0.*df_traj.traj.s_max/500
vtol_view.addTrajectory(df_traj.traj.getPList(df_traj.traj.getP, 0., df_traj.traj.s_max + step, df_traj.traj.s_max/500))

#initialize controllers
low_ctrl = LowLevelControl(M=0.5, Va0=2.0, ts_control=SIM.ts_simulation)
lqr_ctrl = LqrControl(SIM.ts_simulation)
# initialize command message
delta = MsgControls()
vtol._update_true_state()
# initialize the simulation time
sim_time = SIM.start_time
Ts = SIM.ts_simulation

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = vtol.sensors()  # get sensor measurements
    estimated_state = vtol._state  # estimated state is current state
    # ------ Trajectory follower
    desired_state, desired_input = df_traj.desiredState_fromX(estimated_state[0:10])
    #------- High Level controller-------------
    u = lqr_ctrl.update(estimated_state[0:10], desired_state, desired_input, df_traj)
    #------- Low Level Controller -------------
    delta = low_ctrl.update(u[2:5], u[0:2], vtol.true_state)
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

