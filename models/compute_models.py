"""
compute_ss_model
    - Update history:
        2/4/2019 - RWB
        3/13/2024 - RWB
"""
import numpy as np
from tools.rotations import euler_to_quaternion, quaternion_to_euler
from tools.jacobians import jacobian
#from scipy.optimize import minimize
#import parameters.convergence_parameters as VTOL
#from parameters.simulation_parameters import ts_simulation as Ts


def compute_ss_model(vtol, trim_state, trim_input):
    x_euler = euler_state(trim_state)
    A = df_dx(vtol, x_euler, trim_input)
    B = df_du(vtol, x_euler, trim_input)
    return A, B

def euler_state(x_quat: np.ndarray)->np.ndarray:
    '''
        convert (15x1) Quaternion state:
            x_quat = (pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r, r_motor, l_motor)
        to (14x1) Euler state:
            x_euler = (pn, pe, pd, u, v, w, phi, theta, psi, p, q, r, r_motor, l_motor)
    '''
    x_euler = np.zeros((14,1))
    x_euler[0:6] = np.copy(x_quat[0:6])  # copy position, velocity
    phi, theta, psi = quaternion_to_euler(x_quat[6:10])
    x_euler[6,0] = phi
    x_euler[7,0] = theta
    x_euler[8,0] = psi
    x_euler[9:14] = np.copy(x_quat[10:15]) # copy angular rate
    return x_euler

def quaternion_state(x_euler: np.ndarray)->np.ndarray:
    '''
        convert (14x1) Euler state:
            x_euler = (pn, pe, pd, u, v, w, phi, theta, psi, p, q, r, r_motor, l_motor)
        to (15x1) Quaternion state:
            x_quat = (pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r, r_motor, l_motor)
    '''
    x_quat = np.zeros((15,1))
    x_quat[0:6] = np.copy(x_euler[0:6])  # copy position, velocity
    phi = x_euler.item(6)
    theta = x_euler.item(7)
    psi = x_euler.item(8)
    quat = euler_to_quaternion(phi, theta, psi)
    x_quat[6:10] = quat
    x_quat[10:15] = np.copy(x_euler[9:14]) # copy angular rate and motor angles
    return x_quat

def f_euler(vtol, x_euler, input):
    # return 12x1 dynamics (as if state were Euler state)
    # compute f at euler_state
    x_quat = quaternion_state(x_euler)
    vtol._state = x_quat
    vtol._update_velocity_data()
    f = vtol._derivatives(x_quat, vtol._forces_moments(input))
    # f_euler will be f, except for the attitude states
    f_euler_ = euler_state(f)
    q = x_quat[6:10]
    T = jacobian('quaternion_to_euler', q)
    f_euler[6:9] = np.copy(T @ f[6:10])
    return f_euler_

def df_dx(vtol, x_euler, input):
    '''take partial of f_euler with respect to x_euler'''
    eps = 0.01  # deviation
    A = np.zeros((14, 14))  # Jacobian of f wrt x
    f = f_euler(vtol, x_euler, input)
    for i in range(0, 14):
        x_eps = np.copy(x_euler)
        x_eps[i][0] += eps
        f_eps = f_euler(vtol, x_eps, input)
        df = (f_eps - f) / eps
        A[:,i] = df[:,0]
    return A

def df_du(vtol, x_euler, delta):
    '''take partial of f_euler with respect to delta'''
    eps = 0.01  # deviation
    B = np.zeros((14, 8))  # Jacobian of f wrt u
    f = f_euler(vtol, x_euler, delta)
    for i in range(0, 8):
        delta_eps = np.copy(delta)
        delta_eps[i, 0] += eps
        f_eps = f_euler(vtol, x_euler, delta_eps)
        df = (f_eps - f) / eps
        B[:,i] = df[:,0]
    return B

def print_ss_model(filename, A, B, trim_state, trim_input):
    '''write state space model to file'''
    n = B.shape[0]
    m = B.shape[1]

    file = open('models/' + filename, 'w')
    file.write('import numpy as np\n')
    file.write('trim_state = np.array([[')
    for i in range(0,n):
        file.write('%f, ' % trim_state.item(i))  
    file.write(']]).T\n')      
    file.write('trim_input = np.array([[')
    for i in range(0,m):
        file.write('%f, ' % trim_input.item(i))  
    file.write(']]).T\n')      
    file.close()
