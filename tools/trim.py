"""
compute_trim
    - Update history:
        12/29/2018 - RWB
        3/12/2024 - RWB
"""
import numpy as np
from scipy.optimize import minimize
from tools.rotations import euler_to_quaternion, quaternion_to_euler_vec
from tools.jacobians import jacobian
from message_types.vtol.msg_delta import MsgDelta
from models.vtol.vtol_dynamics import VtolDynamics


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

    result = minimize(trim_objective_fun, x0.flatten(), 
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
    Theta = quaternion_to_euler_vec(x_quat[6:10])
    x_euler[6:9] = Theta
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
    # compute x_euler_dot = f_euler(x_euler, input) 
    x_quat = quaternion_state(x_euler)
    vtol._state = x_quat
    vtol._update_velocity_data()
    x_quat_dot = vtol._derivatives(x_quat, vtol._forces_moments(input), input)
    # f_euler will be f, except for the attitude states
    x_euler_dot = euler_state(x_quat_dot)
    q = x_quat[6:10]
    T = jacobian(quaternion_to_euler_vec, q)
    x_euler_dot[6:9] = np.copy(T @ x_quat_dot[6:10])
    return x_euler_dot

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
        delta_eps = delta.to_array()
        delta_eps[i, 0] += eps
        delta_eps_ = MsgDelta()
        delta_eps_.from_array(delta_eps)
        f_eps = f_euler(vtol, x_euler, delta_eps_)
        df = (f_eps - f) / eps
        B[:,i] = df[:,0]
    return B

def print_ss_model(filename, A, B, Va, gamma, trim_state, trim_input):
    '''write state space model to file'''
    n = B.shape[0]
    m = B.shape[1]
    input = trim_input.to_array()
    # open file for writting
    file = open('models/' + filename, 'w')
    file.write('import numpy as np\n')
    # write the airspeed
    file.write('Va = %f\n' % Va)
    # write the flight path angle
    file.write('gamma = %f\n' % gamma)
    # write the trim state
    file.write('trim_state = np.array([[')
    for i in range(0,n):
        file.write('%f, ' % trim_state.item(i))  
    file.write(']]).T\n')      
    # write the trim input
    file.write('trim_input = np.array([[')
    for i in range(0,m):
        file.write('%f, ' % input.item(i))  
    file.write(']]).T\n')      
    # write A
    file.write('A = np.array([\n')
    for i in range(0,n):
        file.write('[')
        for j in range(0,n-1):
            file.write('%f, ' % A[i,j])
        file.write('%f],\n' % A[i,n-1])
    file.write('])\n')      
    # write B
    file.write('B = np.array([')
    for i in range(0,n):
        file.write('[')
        for j in range(0,m-1):
            file.write('%f, ' % B[i,j])
        file.write('%f],\n' % B[i,m-1])
    file.write('])\n')      
    # close file
    file.close()
