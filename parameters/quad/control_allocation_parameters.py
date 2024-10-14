#this file saves and implements the control allocation parameters

import numpy as np

# weighting matrix for minimization of difference between desired and achieved force/torque
K_Wrench = np.diag([1.0, 1.0, 1.0, 1.0, 1.0])

# initial guesses for the actuators and where they should go to.
init_actuators = np.array([0.0, #delta Elevator
                           0.0, #delta Aileron
                           0.0, #delta Rudder
                           0.6, #delta Throttle Forward
                           0.6, #delta Throttle Front Port
                           0.6, #delta Throttle Rear Port
                           0.6, #delta Throttle Rear Starboard
                           0.6])#delta Throttle Front Starboard


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