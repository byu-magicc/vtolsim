"""
    quadsim visualization
    Dean B Anderson: 15/06/2024
"""


import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
# import viewers and video writer
import pyqtgraph as pg
from viewers.quad.quad_viewer import QuadViewer


from tools.rotations import euler_to_rotation
import parameters.quad.simulation_parameters as SIM
from message_types.quad.msg_state import MsgState


# initialize messages
state = MsgState()  # instantiate state message

# initialize viewers and video
plot_app = pg.QtWidgets.QApplication([])
vtol_view = QuadViewer(
    app=plot_app, dt=SIM.ts_simulation,
    plot_period=SIM.ts_plot_refresh)

# initialize the simulation time
sim_time = SIM.start_time
psi = 0
theta = 0
phi = 0
end_time = 1000.0


# main simulation loop
while sim_time < end_time:
    # -------vary states to check viewer-------------
    if sim_time < end_time:
        state.pos[2] += -0.25*SIM.ts_simulation
    elif sim_time < 2*end_time/6:
        state.pos[1] += 1*SIM.ts_simulation
    state.R = euler_to_rotation(phi, theta, psi)

    # -------update viewer and video-------------
    vtol_view.update(state)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

