"""
vtolsim_hover
    - Hover control for VTOL
        2/5/2019 - RWB
        4/2024 - RWB
"""
#/usr/bin/python3
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
import parameters.simulation_parameters as SIM
from viewers.view_manager import ViewManager
from models.vtol_dynamics import VtolDynamics
from tools.signal_generator import SignalGenerator
from message_types.msg_trajectory import MsgTrajectory
from controllers.geometric_controller import GeometricController

#from models.ss_model_Va_0 import trim_state, trim_input
#from controllers.hover_controller.hover_autopilot import HoverController
# from message_types.msg_controls import MsgControls
# from message_types.msg_autopilot import MsgAutopilot
# from message_types.msg_state import MsgState
# from message_types.msg_delta import MsgDelta

#delta = MsgControls()

# initialize elements of the architecture
wind = np.array([[0., 0., 0., 0., 0., 0.]]).T
vtol = VtolDynamics(SIM.ts_simulation)
ctrl = GeometricController(SIM.ts_simulation)
trajectory = MsgTrajectory()
viewers = ViewManager(animation=True, data=True)

# trajectory definition (step response)
pn_command = SignalGenerator(dc_offset=0, amplitude=5, start_time=10.0, frequency = 0.0025)
pe_command = SignalGenerator(dc_offset=0, amplitude=5, start_time=0.0, frequency = 0.0025)
h_command = SignalGenerator(dc_offset=10, amplitude=5, start_time=5.0, frequency = 0.0025)
chi_command = SignalGenerator(dc_offset=np.radians(0), 
                              amplitude=np.radians(45), start_time=5.0, frequency = 0.015)

# vtol._state = trim_state
# delta_trim = MsgDelta()
# delta_trim.from_array(trim_input)

# initialize the simulation time
sim_time = SIM.start_time
end_time = 100.

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < end_time:
    #-------estimator-------------
    estimated_state = vtol.true_state  
    #-------trajectory generator-------------
    trajectory.position = np.array([
            [pn_command.square(sim_time)],
            [pe_command.square(sim_time)],
            [-h_command.square(sim_time)],
        ])
    trajectory.heading = 0.0 #psi_command.square(sim_time)
    #-------controller-------------
    delta, commanded_state = ctrl.update(trajectory, estimated_state)
    #-------physical system-------------
    vtol.update(delta, wind)  # propagate the vtol dynamics
    #-------update viewer-------------
    viewers.update(
        sim_time,
        vtol.true_state,  # true states
        estimated_state,  # estimated states
        vtol.true_state,  # commanded states
        delta,  # inputs to aircraft
        None,  # measurements
    )
    #-------increment time-------------
    sim_time += SIM.ts_simulation
input("Press a key to exit")
