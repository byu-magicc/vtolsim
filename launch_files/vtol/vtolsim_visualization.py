"""
vtolsim
        11/16/2023 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
# import viewers and video writer
import pyqtgraph as pg
from viewers.vtol.vtol_viewer import VtolViewer
from tools.rotations import euler_to_rotation
import parameters.simulation_parameters as SIM
from message_types.msg_state import MsgState

# initialize messages
state = MsgState()  # instantiate state message

# initialize viewers and video
plot_app = pg.QtWidgets.QApplication([])
vtol_view = VtolViewer(
    app=plot_app, dt=SIM.ts_simulation,
    plot_period=SIM.ts_plot_refresh)

# initialize the simulation time
sim_time = SIM.start_time
psi = 0
theta = 0
phi = 0
end_time = 4000
# main simulation loop
while sim_time < end_time:
    # -------vary states to check viewer-------------
    if sim_time < end_time/8:
        state.pos[0] += 10*SIM.ts_simulation
    elif sim_time < 2*end_time/8:
        state.pos[1] += 10*SIM.ts_simulation
    elif sim_time < 3*end_time/8:
        state.pos[2] -= 10*SIM.ts_simulation
    elif sim_time < 4*end_time/8:
        psi += 0.01*SIM.ts_simulation
    elif sim_time < 5*end_time/8:
        theta += 0.01*SIM.ts_simulation
    elif sim_time < 6*end_time/8:
        phi += 0.01*SIM.ts_simulation
    elif sim_time < 7*end_time/8:
        state.motor_angle[0,0] += 0.01*SIM.ts_simulation  # right motor
    else:
        state.motor_angle[1,0] += 0.01*SIM.ts_simulation  # left motor
    state.R = euler_to_rotation(phi, theta, psi)

    # -------update viewer and video-------------
    vtol_view.update(state)

    # -------increment time-------------
    sim_time += SIM.ts_simulation




