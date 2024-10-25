"""
compute_trim 
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        12/29/2018 - RWB
"""
import numpy as np
from scipy.optimize import minimize
from message_types.msg_delta import MsgDelta
from tools.rotations import euler_to_quaternion

def compute_trim(mav, Va: float, gamma: float):
    '''
        define initial state and input
        set the initial conditions of the optimization
    '''
    e0 = euler_to_quaternion(0., gamma, 0.)
    state0 = np.array([[mav._state.item(0)],  # pn
                   [mav._state.item(1)],  # pe
                   [mav._state.item(2)],  # pd
                   [Va],  # u
                   [0.],  # v
                   [0.],  # w
                   [e0.item(0)],  # e0
                   [e0.item(1)],  # e1
                   [e0.item(2)],  # e2
                   [e0.item(3)],  # e3
                   [0.],  # p
                   [0.],  # q
                   [0.]   # r
                   ])
    delta0 = MsgDelta(elevator=0., aileron=0., rudder=0., forwardThrottle=0.5)
    print("delta0.to_array()[0:4]: " , delta0.to_array()[0:4])
    x0 = np.concatenate((state0, delta0.to_array()[0:4]), axis=0)
    # define equality constraints
    cons = ({'type': 'eq',
             'fun': lambda x: np.array([
                                x[3]**2 + x[4]**2 + x[5]**2 - Va**2,  # magnitude of velocity vector is Va
                                x[4],  # v=0, force side velocity to be zero
                                x[6]**2 + x[7]**2 + x[8]**2 + x[9]**2 - 1.,  # force quaternion to be unit length
                                x[7],  # e1=0  - forcing e1=e3=0 ensures zero roll and zero yaw in trim
                                x[9],  # e3=0
                                x[10],  # p=0  - angular rates should all be zero
                                x[11],  # q=0
                                x[12],  # r=0
                                ]),
             'jac': lambda x: np.array([
                                [0., 0., 0., 2*x[3], 2*x[4], 2*x[5], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 2*x[6], 2*x[7], 2*x[8], 2*x[9], 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
                                ])
             })
    # solve the minimization problem to find the trim states and inputs

    res = minimize(trim_objective_fun, x0, method='SLSQP', args=(mav, Va, gamma),
                   constraints=cons, 
                   options={'ftol': 1e-10, 'disp': True})
    # extract trim state and input and return
    trim_state = np.array([res.x[0:13]]).T
    trim_input = MsgDelta(elevator=res.x.item(13),
                          aileron=res.x.item(14),
                          rudder=res.x.item(15),
                          forwardThrottle=res.x.item(16))
    trim_input.print()
    print('trim_state=', trim_state.T)
    return trim_state, trim_input


def trim_objective_fun(x: np.ndarray, mav, Va: float, gamma: float) -> float:
    # objective function to be minimized
    state = np.reshape(x[0:13], (13, 1))
    delta = MsgDelta(elevator=x.item(13),
                     aileron=x.item(14),
                     rudder=x.item(15),
                     forwardThrottle=x.item(16))
    desired_trim_state_dot = np.array([[0., 0., -Va*np.sin(gamma), 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]]).T
    mav._state[0:13] = state
    mav._state[13,0] = 0
    mav._state[14,0] = 0
    mav._update_velocity_data()
    forces_moments = mav._forces_moments(delta)
    f = mav._f(state, forces_moments)
    tmp = desired_trim_state_dot - f
    J = np.linalg.norm(tmp[2:13])**2.0
    return J
