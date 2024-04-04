"""
vtolsim_control_allocation
    - Update history:
        2022 - Jacob Willis and Mason Peterson
        4/2024 - RWB
"""
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
import parameters.simulation_parameters as SIM
from models.vtol_dynamics import VtolDynamics
from viewers.view_manager import ViewManager
from message_types.msg_delta import MsgDelta
from tools.signals import SignalGenerator

#from message_types.msg_controls import msgControls
#from dynamics.compute_models import compute_tf_model
#from dynamics.trim import *
from controllers.rate_control import RateControl
from controllers.attitude_control import AttitudeControl
from control_allocation.control_allocation_cls import ControlAllocation
#from tools.msg_convert import *
#import time

# initialize elements of the architecture
wind = np.array([[0., 0., 0., 0., 0., 0.]]).T
vtol = VtolDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)

# # trim
# Va_star = 0.0
# gamma_star = 0.0
# servo0 = np.radians(90)
# state_trim, delta_trim = compute_trim(vtol, Va_star, gamma_star, servo0 = np.radians(90))
# vtol._state = state_trim
# vtol._update_true_state()

#initialize low level control
att_ctrl = AttitudeControl(ts_control=SIM.ts_simulation)
rate_control = RateControl(ts_control=SIM.ts_simulation)
control_alloc = ControlAllocation(servo0=np.radians(90))

# signal generators
p_command = SignalGenerator(
    dc_offset=np.radians(0), 
    amplitude=np.radians(15), 
    start_time=0.0, 
    frequency = 0.1)
q_command = SignalGenerator(
    dc_offset=np.radians(0), 
    amplitude=np.radians(15), 
    start_time=0.0, 
    frequency = 0.1)
r_command = SignalGenerator(
    dc_offset=np.radians(0), 
    amplitude=np.radians(15), 
    start_time=0.0, 
    frequency = 0.1)

phi_command = SignalGenerator(
    dc_offset=np.radians(0), 
    amplitude=np.radians(15), 
    start_time=0.0, 
    frequency = 0.1)
theta_command = SignalGenerator(
    dc_offset=np.radians(0), 
    amplitude=np.radians(15), 
    start_time=0.0, 
    frequency = 0.1)
psi_command = SignalGenerator(
    dc_offset=np.radians(0), 
    amplitude=np.radians(15), 
    start_time=0.0, 
    frequency = 0.1)

# initialize the simulation time
sim_time = SIM.start_time
Ts = SIM.ts_simulation

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = vtol.sensors()  # get sensor measurements
    estimated_state = vtol._state  # estimated state is current state
    
    #-------controller-------------
    commanded_state = vtol.true_state  # commanded state is current state
    att_cmd = np.array(
        [0., #np.array([phi_command.square(sim_time)
        0.0, #theta_command.square(sim_time),
        0.0] #psi_command.square(sim_time)]
    )
    omega_d = att_ctrl.update(att_cmd, vtol.true_state).reshape(-1)
    omega = vtol._state[10:13]
    tau_d = rate_control.update(omega_d, omega)
    delta_old = control_alloc.update(np.array([0., -9.81]), tau_d, vtol._Va)
    delta = MsgDelta(delta_old)
    
    #-------update physical system-------------
    vtol.update(delta, wind)  # propagate the VTOL dynamics

    #-------update viewers-------------
    viewers.update(
        sim_time,
        vtol.true_state,  # true states
        vtol.true_state,  # estimated states
        vtol.true_state,  # commanded states
        delta,  # inputs to aircraft
        None,  # measurements
    )

    #-------increment time-------------
    sim_time += Ts

input("Press a key to exit")
