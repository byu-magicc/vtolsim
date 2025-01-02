#this file implements the parameters used for control allocation for this simulation environment

#this file saves and implements the control allocation parameters

import numpy as np


scaling = 1.0

Fx_bar = 100.0
Fz_bar = 100.0
Mx_bar = 50.0
My_bar = 50.0
Mz_bar = 50.0



# weighting matrix for minimization of difference between desired and achieved force/torque
K_Wrench = scaling*np.diag([1/(Fx_bar**2), 1/(Fz_bar**2), 1/(Mx_bar**2), 1/(My_bar**2), 1/(Mz_bar**2)])

# initial guesses for the actuators and where they should go to.
init_actuators = np.array([0.0, #delta Elevator
                           0.0, #delta Aileron
                           0.0, #delta Rudder
                           0.6, #delta Throttle Forward
                           0.6, #delta Throttle Front Port
                           0.6, #delta Throttle Rear Port
                           0.6, #delta Throttle Rear Starboard
                           0.6])#delta Throttle Front Starboard


#initial guesses for the airplane actuators
airplaneInit_actuators = np.array([0, #Initial Elevator
                                   0, #Initial Aileron
                                   0, #Initial rudder
                                   0.6, #Initial Forward Prop
                                   0, #initial v1
                                   0, #initial v2
                                   0, #initial v3
                                   0]) #initial v4


#creates the initial guess for the airplane control
init_plane_control = init_actuators[0:4]
#creates the initial guess for the quadrotor control
init_quad_control = init_actuators[4:8]

# minimum-energy actuator setpoints
actuators_desired = np.zeros(8)

#minimum energy surfaces actuator setpoints
actuators_surfaces_desired = np.zeros(4)

#minimum energy quadrotor actuator setpoints
actuators_quadrotors_desired = np.zeros(4)

# max iterations for nonlinear solver
max_iter = 50



#saves the bounds for finding the delta_c stuff
actuatorBounds_delta_c = [(-1.0, 1.0), #delta elevator
                         (-1.0, 1.0), #delta aileron
                         (-1.0, 1.0), #delta rudder 
                         (0.0, 1.0), #delta throttle forward
                         (0.0, 0.0), #delta throttle vertical 1
                         (0.0, 0.0), #delta throttle vertical 2
                         (0.0, 0.0), #delta throttle vertical 3
                         (0.0, 0.0)] #delta throttle vertical 4

#saves thr bounds for finding the delta_t portion.
#just the four quadrotor bounds
actuatorBounds_delta_t = [(0.0, 1.0), #delta throttle vertical 1
                         (0.0, 1.0), #delta throttle vertical 2
                         (0.0, 1.0), #delta throttle vertical 3
                         (0.0, 1.0)] #delta throttle vertical 4


#saves the bounds for finding the complete delta
actuatorBounds = [(-1.0, 1.0), #delta elevator
                  (-1.0, 1.0), #delta aileron
                  (-1.0, 1.0), #delta rudder 
                  (0.0, 1.0), #delta throttle forward
                  (0.0, 1.0), #delta throttle vertical 1
                  (0.0, 1.0), #delta throttle vertical 2
                  (0.0, 1.0), #delta throttle vertical 3
                  (0.0, 1.0)] #delta throttle vertical 4