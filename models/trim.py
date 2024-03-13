"""
compute_trim
    - Update history:
        12/29/2018 - RWB
        3/12/2024 - RWB
"""
import numpy as np
from scipy.optimize import minimize
from tools.rotations import euler_to_quaternion
from message_types.msg_delta import MsgDelta
from models.vtol_dynamics import VtolDynamics

def compute_trim(vtol: VtolDynamics, 
                 Va: float, 
                 gamma: float, 
                 motor0: float=0., 
                 motor_range: float=np.radians(55),
                 )->tuple[np.ndarray, MsgDelta]:
    # define initial state and input
    quat = euler_to_quaternion(0., gamma, 0.)
    state0 = np.array([
        [vtol._state.item(0)],  # pn 0
        [vtol._state.item(1)],  # pe 1
        [vtol._state.item(2)],  # pd 2
        [vtol._Va + 1.],  # u 3 - +1 to handle Va = 0.
        [0.],  # v 4
        [0.],  # w 5
        [quat.item(0)],  # e0 6
        [quat.item(1)],  # e1 7
        [quat.item(2)],  # e2 8
        [quat.item(3)],  # e3 9
        [0.],  # p 10
        [0.],  # q 11
        [0.],  # r 12
        [motor0],  # right rotor 13
        [motor0]   # left rotor 14
        ])
    delta0 = np.array([[0.],  # elevator 15
                       [0.],  # aileron 16
                       [0.],  # rudder 17
                       [1.0],  # throttle_right 18
                       [1.0],  # throttle_left 19
                       [0.0],  # throttle_rear 20
                       [motor0], # motor_right 21
                       [motor0]  # motor_left 22
                       ])
    x0 = np.concatenate((state0, delta0), axis=0)
    # define equality constraints
    cons = ([{'type': 'eq',
             'fun': lambda x: np.array([
                x[3]**2 + x[4]**2 + x[5]**2 - Va**2,  # magnitude of velocity is Va
                x[4],  # v=0, force side velocity to be zero
                x[6]**2 + x[7]**2 + x[8]**2 + x[9]**2 - 1.,  #  quaternion is unit length
                x[7], # e1=0  - forcing e1=e3=0 ensures zero roll and zero yaw in trim
                x[9], # e3=0
                x[10], # p=0  - angular rates should all be zero
                x[11], # q=0
                x[12], # r=0
                x[13]-x[21], # right rotor=right rotor command
                x[14]-x[22],  # left rotor=left rotor command
                x[17], # rudder is zero
                ]),
             'jac': lambda x: np.array([
                [0., 0., 0., 2*x[3], 2*x[4], 2*x[5], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 2*x[6], 2*x[7], 2*x[8], 2*x[9], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., -1., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., -1.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
                ])
             }, {'type': 'ineq', #>0
            'fun': lambda x: np.array([
                x[18], #3 motor inputs between 0 and 1
                x[19],
                x[20],
                -x[18]+1.0,
                -x[19]+1.0,
                -x[20]+1.0,
                x[21]+np.radians(15.0),  #tilt servos between -15 and 120 degrees
                -x[21]+np.radians(120.0),
                x[22]+np.radians(15.0),
                -x[22]+np.radians(120.0),
                (x[21]- motor0)+motor_range,  # keep motors within 15 degrees of motor0
                -(x[21]- motor0)+ motor_range,
                (x[22]-motor0)+motor_range,  # keep motors within 15 degrees of motor0
                -(x[22]-motor0)+motor_range,
                x[6], # quaternion e0>0
                x[6]-np.cos(np.radians(90)), # pitch angle limited to 15 degrees
                ]),
            'jac': lambda x: np.array([
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1.],
                [0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                ])
             }])
    # solve the minimization problem to find the trim states and inputs

    result = minimize(trim_objective_fun, 
                      x0, 
                      method='SLSQP', 
                      args = (vtol, Va, gamma),
                      constraints=cons, 
                      options={'ftol': 1e-10, 'disp': True, 'maxiter': 1000})
    J = trim_objective_fun(result.x, vtol, Va, gamma)
    # extract trim state and input and return
    trim_state = np.array([result.x[0:15]]).T
    trim_state[0,0] =  vtol._state.item(0)  # pn 0
    trim_state[1,0] =  vtol._state.item(1)  # pe 1
    trim_state[2,0] = vtol._state.item(2)  # pd 2
    trim_input = np.array([result.x[15:23]]).T
    print('trim_state=', trim_state.T)
    print('trim_input=', trim_input.T)
    print('J_final=', J)

    trim_delta = MsgDelta()
    trim_delta.from_array(trim_input)
    return trim_state, trim_delta

# objective function to be minimized
def trim_objective_fun(x, vtol, Va, gamma):
    state = np.array([x[0:15]]).T
    delta = MsgDelta()
    delta.elevator = x.item(15)
    delta.aileron = x.item(16)
    delta.rudder = x.item(17)
    delta.throttle_right = x.item(18)
    delta.throttle_left = x.item(19)
    delta.throttle_rear = x.item(20)
    delta.motor_right = x.item(21)
    delta.motor_left = x.item(22)
    xdot = np.array([[9999., 9999., -Va*np.sin(gamma), 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]]).T
    vtol._state = state
    vtol._update_velocity_data()
    forces_moments = vtol._forces_moments(delta)
    f = vtol._derivatives(state, forces_moments, delta)
    tmp = xdot - f
    J = np.linalg.norm(tmp[2:15])**2.0
    return J
