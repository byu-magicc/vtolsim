"""
vtolsim
    - Update history:  
        5/4/2019 - RWB
        1/31/2024 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
# import viewers and video writer
import pyqtgraph as pg
import numpy as np
from viewers.vtol_viewer import vtolViewer
from tools.rotations import euler_to_rotation
import parameters.simulation_parameters as SIM
from vtolsim.message_types.msg_state_old import MsgState

# initialize messages
state = MsgState()  # instantiate state message

# initialize viewers
# initialize viewers and video
plot_app = pg.QtWidgets.QApplication([])
vtol_view = vtolViewer(
    app=plot_app, dt=SIM.ts_simulation,
    plot_period=SIM.ts_plot_refresh)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
while sim_time < SIM.end_time:
    #-------vary states to check viewer-------------
    if sim_time < SIM.end_time/6:
        state.pn += 10*SIM.ts_simulation
    elif sim_time < 2*SIM.end_time/6:
        state.pe += 10*SIM.ts_simulation
    elif sim_time < 3*SIM.end_time/6:
        state.h += 10*SIM.ts_simulation
    elif sim_time < 4*SIM.end_time/6:
        state.psi += 0.1*SIM.ts_simulation
    elif sim_time < 5*SIM.end_time/6:
        state.theta += 0.1*SIM.ts_simulation
    else:
        state.phi += 0.1*SIM.ts_simulation
    state.right_rotor = (np.pi/2) * np.sin(0.1*sim_time)
    state.left_rotor = (np.pi/2) * np.cos(0.1*sim_time)

    #-------update viewer and video-------------
    vtol_view.update(state)

    #-------increment time-------------
    sim_time += SIM.ts_simulation

print("Press Ctrl-Q to exit...")



