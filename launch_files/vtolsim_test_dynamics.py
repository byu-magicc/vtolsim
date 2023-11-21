"""
edit history
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
from message_types.msg_delta import MsgDelta

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
delta = MsgDelta()

# initialize the simulation time
sim_time = SIM.start_time
# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------vary forces and moments to check dynamics-------------
    delta.thrust = VTOL.mass * VTOL.gravity + 0.0
    delta.torque = np.array([[0.], [0.], [0.]])
    # -------physical system-------------
    wind = np.array([[0.], [0.], [0.], [0.], [0.], [0.]])
    vtol.update(delta, wind)
    # -------update viewer-------------
    vtol_view.update(vtol.state) 
    data_view.update(vtol.state,  # true states
                     vtol.state,  # estimated states
                     vtol.state,  # commanded states
                     delta)  # inputs to the vtol
    # -------increment time-------------
    sim_time += SIM.ts_simulation





