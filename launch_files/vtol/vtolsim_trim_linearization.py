"""
vtolsim_trim_level
    - compute trim for level flight and associated state space equations
    - verify that the eVTOL remains in trim
    - Update history:
        3/12/2024 - RWB
"""
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
import parameters.simulation_parameters as SIM
from models.vtol_dynamics import VtolDynamics
from tools.trim import compute_trim, compute_ss_model, print_ss_model
from viewers.view_manager import ViewManager

# initialize elements of the architecture
vtol = VtolDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)

# compute trim and state space models at zeros airspeed (hover)
Va = 0.
gamma = np.radians(0)
motor_angles0 = np.radians(90)
trim_state, trim_delta = compute_trim(vtol, Va, gamma, motor0=motor_angles0)
vtol._state = trim_state  # set the initial state of the vtol to the trim state
delta = trim_delta  # set input to constant constant trim input
A, B = compute_ss_model(vtol, trim_state, trim_delta)
print_ss_model('ss_model_Va_0.py', A, B, Va, gamma, trim_state, trim_delta)

# compute trim and state space models at Va=5
Va = 5.
gamma = np.radians(0)
motor_angles0 = np.radians(70)
trim_state, trim_delta = compute_trim(vtol, Va, gamma, motor0=motor_angles0)
vtol._state = trim_state  # set the initial state of the vtol to the trim state
delta = trim_delta  # set input to constant constant trim input
A, B = compute_ss_model(vtol, trim_state, trim_delta)
print_ss_model('ss_model_Va_5.py', A, B, Va, gamma, trim_state, trim_delta)

# compute trim and state space models at Va=10
Va = 10.
gamma = np.radians(0)
motor_angles0 = np.radians(30)
trim_state, trim_delta = compute_trim(vtol, Va, gamma, motor0=motor_angles0)
vtol._state = trim_state  # set the initial state of the vtol to the trim state
delta = trim_delta  # set input to constant constant trim input
A, B = compute_ss_model(vtol, trim_state, trim_delta)
print_ss_model('ss_model_Va_10.py', A, B, Va, gamma, trim_state, trim_delta)

# compute trim and state space models at Va=15 (level)
Va = 15.
gamma = np.radians(0)
motor_angles0 = np.radians(0)
trim_state, trim_delta = compute_trim(vtol, Va, gamma, motor0=motor_angles0)
vtol._state = trim_state  # set the initial state of the vtol to the trim state
delta = trim_delta  # set input to constant constant trim input
A, B = compute_ss_model(vtol, trim_state, trim_delta)
print_ss_model('ss_model_Va_15.py', A, B, Va, gamma, trim_state, trim_delta)


# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------update physical system-------------
    current_wind = np.zeros((6, 1))
    vtol.update(delta, current_wind)  

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
    sim_time += SIM.ts_simulation




