#this file implements the simulation file which will optimally allocate our tiltrotor's
#state in both multicopter and fixed wing mode. We will be using this in such a manner that 
#it minimizes the time and energy expended in multicopter mode. We're only making money when
#we're moving, and we're only moving quickly in fixed wing mode.

#our formatting and methodology will be the same as with the vtolsim_lqr.py file, which currently
#works correctly

#imports os and system
import os
import sys

#imports path so we can look outside our present folder
from pathlib import Path
sys.path.insert(0, os.fspath(Path(__file__).parents[1]))

#imports numpy
import numpy as np

#gets the simulation parameters
import parameters.simulation_parameters as SIM
#gets the spline parameters, which I presume are for spline path following
import parameters.spline_parameters as SPLP
#imports the msg convert functions
from message_types.msg_convert import *
#gets vtoldynamics
from models.vtol_dynamics import VtolDynamics
#imports the wind simulation
from models.wind_simulation import WindSimulation

###########################################################################
#imports the various controllers for this one
#gets lqr control
from controllers.lqr.lqr_control import LqrControl
#imports low level control
from controllers.low_level_control import LowLevelControl
#imports the rate controller
from controllers.rate_control import RateControl
#imports the attitude controller
from controllers.attitude_control import AttitudeControl

#imports the control allocation function
#TODO need to implement this
###########################################################################

#imports spline trajectories
from planners.trajectory.spline_trajectory import SplineTrajectory
#imports differential flatness, whatever that is
from planners.trajectory.differential_flatness import DifferentialFlatness
#imports msg delta, or control commands (aileron, elevator, rudder, and throttle)
from message_types.msg_delta import MsgDelta
#imports the ViewManager, which does all our animations and dataplotting
from viewers.view_manager import ViewManager
#gets the compute trim function
from tools.trim import compute_trim

#imports the signals function
from tools.signals import Signals




#sets the wind vector
wind = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T
#gets the Vtol Dynamics
vtol = VtolDynamics(SIM.ts_simulation)
#uses the viewer from view manager, which one actually works
viewers = ViewManager(animation=True, data=True)
#initializes the wind simulation
wind = WindSimulation()


#Trim setup
#this section gets and sets up the trim
Va_star = 0.0
gamma_star = 0.0
#I believe that servo0 represents the resting position of the tilt servos
servo0 = np.radians(90)
#gets the trim using the above settings
state_trim, delta_trim = compute_trim(vtol, Va_star, gamma_star, servo0 = np.radians(90))

#sets the state of the vtol to the trim state
vtol._state = state_trim
#updates the true state of the system using the built in function
vtol._update_true_state()

#initializes the low level control of the system
#gets the attitude controller
attitudeControl = AttitudeControl(ts_control=SIM.ts_simulation)
#initializes the rate controller
rateControl = RateControl(ts_control=SIM.ts_simulation)
#initializes the control allocation function
controlAllocation = controlAllocation(servo0=servo0)


###########################################################################
#sets the p, q, and r signal commands variables

#sets the dc offset
p_dc_offset = np.radians(0.0)
q_dc_offset = np.radians(0.0)
r_dc_offset = np.radians(0.0)

phi_dc_offset = 0.0
theta_dc_offset = 0.0
psi_dc_offset = 0.0


#sets the amplitudes
p_amplitude = np.radians(15)
q_amplitude = np.radians(15)
r_amplitude = np.radians(15)

phi_amplitude = np.radians(15)
theta_amplitude = np.radians(15)
psi_amplitude = np.radians(15)

#sets the signal frequencies
p_frequency = 0.1
q_frequency = 0.1
r_frequency = 0.1

phi_frequency = 0.0
theta_frequency = 0.0
psi_frequency = 0.0
###########################################################################


#instantiates the signal generators
#gets the p command
p_command = Signals(dc_offset=p_dc_offset, amplitude=p_amplitude, start_time=0.0, frequency=p_frequency)
q_command = Signals(dc_offset=q_dc_offset, amplitude=q_amplitude, start_time=0.0, frequency=q_frequency)
r_command = Signals(dc_offset=r_dc_offset, amplitude=r_amplitude, start_time=0.0, frequency=r_frequency)

phi_command = Signals(dc_offset=phi_dc_offset, amplitude=phi_amplitude, start_time=0.0, frequency=phi_frequency)
theta_command = Signals(dc_offset=theta_dc_offset, amplitude=theta_amplitude, start_time=0.0, frequency=theta_frequency)
psi_command = Signals(dc_offset=psi_dc_offset, amplitude=psi_amplitude, start_time=0.0, frequency=psi_frequency)


#initializes the start times
sim_time = SIM.start_time
#sets the time sampling of the simulation
Ts = SIM.ts_simulation

#creates the main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    
    #---------Observer------------
    #uses the observer to get the appropriate measurements for the system
    #first gets the measurements of the system
    measurements = vtol.sensors()
    #gets the estimated states actually in this first instance using the true states
    #TODO in the future, we will need to update this to use a Kalman filter for the observer
    estimated_state = vtol.true_state
    
    #-----controller-----------------
    #sets the commanded state to be the current state? Why are we doing this 
    #TODO: by TODO, I mean I need to figure this thing out. Why are we setting it this way?
    commanded_state = vtol.true_state

    #sets the attitude command, which will be phi, theta, and psi
    #TODO make sure whatever is reading this array is looking for a 2-D vector as a column vector
    attitude_command = np.array([[phi_command],
                                 [theta_command],
                                 [psi_command]])
    
    
    #calls the attitude control function.
    #takes as an argument the attitude command and vtol state (estimated or true)
    #returns (p_command, q_command, r_command), which are roll, pitch, and yaw rates
    pqr_command = attitudeControl.update(attitude_command, vtol.true_state).reshape(-1)

    #gets the actual pqr vecor
    pqr = vtol._state[10:13]

    #calls the rate controller, which returns the tau_d, which is the desired torques
    #TODO find out what tau means
    #takes as arguments the pqr command from the attitude controller and the actual pqr
    tau_d = rateControl.update(pqr_command, pqr)

    #calls the control allocation function, which takes as an argument the tau and returns the 
    #deltas for all of the control surfaces
    #TODO fix the gravity array thing here
    delta = controlAllocation.update(np.array([0.0, -9.81]), tau_d, vtol._Va)

    #-------Updates the physical system ------------
    #gets the current wind
    current_wind = wind.update()
    #gets the vtol update using the delta and the state, I think
    vtol.update(delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T)

    #---------update viewers-----------------
    #updates the 






