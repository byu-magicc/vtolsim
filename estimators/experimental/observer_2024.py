"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
        3/4/2024 - RWB
"""
import numpy as np
from scipy import stats
import parameters.control_parameters_airplane as CTRL
import parameters.sensor_parameters as SENSOR
from tools.wrap import wrap
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from estimators.filters import AlphaFilter

class Observer:
    def __init__(self, ts: float, initial_measurements: MsgSensors=MsgSensors()):
        self.Ts = ts  # sample rate of observer
        # initialized estimated state message
        self.estimated_state = MsgState()
        # use alpha filters to low pass filter gyros and accels
        # alpha = Ts/(Ts + tau) where tau is the LPF time constant
        self.lpf_gyro_x = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_x)
        self.lpf_gyro_y = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_y)
        self.lpf_gyro_z = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_z)
        self.lpf_accel_x = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_x)
        self.lpf_accel_y = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_y)
        self.lpf_accel_z = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_z)
        # use alpha filters to low pass filter absolute and differential pressure
        self.lpf_abs = AlphaFilter(alpha=0.9, y0=initial_measurements.abs_pressure)
        self.lpf_diff = AlphaFilter(alpha=0.7, y0=initial_measurements.diff_pressure)
        # ekf for phi and theta
        self.attitude_ekf = EkfAttitude(self.Ts)
        # ekf for pn, pe, Vg, chi, wn, we, psi
        self.position_ekf = EkfPosition(self.Ts)

    def update(self, measurement: MsgSensors) -> MsgState:
        # estimates for p, q, r are low pass filter of gyro minus bias estimate
        self.estimated_state.p = self.lpf_gyro_x.update(measurement.gyro_x) - self.estimated_state.bx
        self.estimated_state.q = self.lpf_gyro_y.update(measurement.gyro_y) - self.estimated_state.by
        self.estimated_state.r = self.lpf_gyro_z.update(measurement.gyro_z) - self.estimated_state.bz
        # invert sensor model to get altitude and airspeed
        abs_pressure = self.lpf_abs.update(measurement.abs_pressure)
        diff_pressure = self.lpf_diff.update(measurement.diff_pressure)
        self.estimated_state.altitude = abs_pressure/CTRL.rho/CTRL.gravity
        self.estimated_state.Va = np.sqrt(2 * diff_pressure / CTRL.rho)
        # estimate phi and theta with simple ekf
        self.attitude_ekf.update(measurement, self.estimated_state)
        # estimate pn, pe, Vg, chi, wn, we, psi
        self.position_ekf.update(measurement, self.estimated_state)
        # not estimating these
        self.estimated_state.alpha = self.estimated_state.theta
        self.estimated_state.beta = 0.0
        self.estimated_state.bx = 0.0
        self.estimated_state.by = 0.0
        self.estimated_state.bz = 0.0
        return self.estimated_state


class EkfAttitude:
    '''
    implement continous-discrete EKF to estimate roll and pitch angles
    '''
    def __init__(self, ts):
        self.Q = np.diag([
            (1e-3)**2, # phi 
            (1e-3)**2, # theta
            ])
        self.P = np.diag([
            (10*np.pi/180.)**2, # phi
            (10*np.pi/180.)**2, # theta
            ])
        self.xhat = np.array([
            [0.*np.pi/180.], # phi 
            [0.*np.pi/180.], # theta
            ]) 
        self.Q_gyro = SENSOR.gyro_sigma**2 * np.diag([1.0, 1.0, 1.0])
        self.R_accel = SENSOR.accel_sigma ** 2 * np.diag([1.0, 1.0, 1.0])
        self.N = 5  # number of prediction step per sample
        self.Ts = ts/self.N
        self.gate_threshold = stats.chi2.isf(q=0.1, df=2)

    def update(self, measurement: MsgSensors, state: MsgState):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.phi = self.xhat.item(0)
        state.theta = self.xhat.item(1)

    def f(self, x: np.ndarray, measurement: MsgSensors, state: MsgState) -> np.ndarray:
        # system dynamics for propagation model: xdot = f(x, u)
        phi = x.item(0)
        theta = x.item(1)
        p = measurement.gyro_x - state.bx
        q = measurement.gyro_y - state.by
        r = measurement.gyro_z - state.bz
        G = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                      [0.0, np.cos(phi), -np.sin(phi)]])
        xdot = G @ np.array([[p], [q], [r]])
        return xdot

    def h(self, x: np.ndarray, measurement: MsgSensors, state: MsgState)->np.ndarray:
        # measurement model y=h(x,u)
        phi = x.item(0)
        theta = x.item(1)
        p = measurement.gyro_x - state.bx
        q = measurement.gyro_y - state.by
        r = measurement.gyro_z - state.bz
        Va = state.Va #np.sqrt(2 * measurement.diff_pressure / CTRL.rho)
        y = np.array([
            [q * Va * np.sin(theta) + CTRL.gravity * np.sin(theta)],  # x-accel
            [r * Va * np.cos(theta) - p * Va * np.sin(theta) - CTRL.gravity * np.cos(theta) * np.sin(phi)],
            # y-accel
            [-q * Va * np.cos(theta) - CTRL.gravity * np.cos(theta) * np.cos(phi)],  # z-accel
        ])
        return y

    def propagate_model(self, measurement: MsgSensors, state: MsgState):
        # model propagation
        Tp = self.Ts
        for i in range(0, self.N):
            phi = self.xhat.item(0)
            theta = self.xhat.item(1)
            # propagate model
            self.xhat = self.xhat + Tp * self.f(self.xhat, measurement, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
            # compute G matrix for gyro noise
            G = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                          [0.0, np.cos(phi), -np.sin(phi)]])
            # convert to discrete time models
            A_d = np.eye(2) + Tp * A + ((Tp ** 2)/2.) * A @ A
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + Tp**2 * (self.Q + G @ self.Q_gyro @ G.T)

    def measurement_update(self, measurement: MsgSensors, state: MsgState):
        # measurement updates
        yhat = self.h(self.xhat, measurement, state)
        C = jacobian(self.h, self.xhat, measurement, state)
        y = np.array([[measurement.accel_x, measurement.accel_y, measurement.accel_z]]).T
        S_inv = np.linalg.inv(self.R_accel + C @ self.P @ C.T)
        if True: #(y-h).T @ S_inv @ (y-h) < self.gate_threshold:
            L = self.P @ C.T @ S_inv
            tmp = np.eye(2) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R_accel @ L.T
            self.xhat = self.xhat + L @ (y - yhat)
            # print('updating')


class EkfPosition:
    # implement continous-discrete EKF to estimate pn, pe, Vg, chi, wn, we, psi
    def __init__(self, ts):
        self.Q = np.diag([
                    (0.03)**2,  # pn
                    (0.03)**2,  # pe
                    (0.03)**2,  # Vg
                    (0.03)**2, # chi
                    (0.03)**2, # wn
                    (0.03)**2, # we
                    (0.03)**2, # psi
                    ])
        self.P = np.diag([(1.0)**2, # pn
                          (1.0)**2, # pe
                          (1.0)**2, # Vg
                          (30.*np.pi/180.)**2, # chi
                          (10.0)**2, # wn
                          (10.0)**2, # we
                          (30.*np.pi/180.)**2, # psi
                          ])
        self.xhat = np.array([
            [0.0], # pn 
            [0.0], # pe 
            [25.0], # Vg 
            [0.0], # chi
            [0.0], # wn 
            [0.0], # we 
            [0.0], # psi
            ])
        self.R_gps = np.diag([
                    SENSOR.gps_n_sigma**2,  # y_gps_n
                    SENSOR.gps_e_sigma**2,  # y_gps_e
                    SENSOR.gps_Vg_sigma**2,  # y_gps_Vg
                    SENSOR.gps_course_sigma**2,  # y_gps_course
                    ])
        self.R_pseudo = np.diag([
                    0.0001,  # pseudo measurement #1
                    0.0001,  # pseudo measurement #2
                    ])
        self.N = 10  # number of prediction step per sample
        self.Ts = (ts / self.N)
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999
        self.pseudo_threshold = stats.chi2.isf(q=0.1, df=2)
        #self.gps_threshold = stats.chi2.isf(q=0.01, df=4)
        self.gps_threshold = 100000 # don't gate GPS

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.north = self.xhat.item(0)
        state.east = self.xhat.item(1)
        state.Vg = self.xhat.item(2)
        state.chi = self.xhat.item(3)
        state.wn = self.xhat.item(4)
        state.we = self.xhat.item(5)
        state.psi = self.xhat.item(6)

    def f(self, x, measurement, state):
        # system dynamics for propagation model: xdot = f(x, u)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)
        Va = state.Va #np.sqrt(2 * measurement.diff_pressure / CTRL.rho)
        q = measurement.gyro_y
        r = measurement.gyro_z
        # psidot = (state.q * np.sin(state.phi) + state.r * np.cos(state.phi)) / np.cos(state.theta)
        # Vgdot = ((state.Va * np.cos(psi) + wn) * (-psidot * state.Va * np.sin(psi))
        #          + (state.Va * np.sin(psi) + we) * (psidot * state.Va * np.cos(psi))) / Vg
        psidot = (q * np.sin(state.phi) + r * np.cos(state.phi)) / np.cos(state.theta)
        Vgdot = ((Va* np.cos(psi) + wn) * (-psidot * Va * np.sin(psi))
                 + (Va * np.sin(psi) + we) * (psidot * Va * np.cos(psi))) / Vg
        xdot = np.array([[Vg * np.cos(chi)],
                        [Vg * np.sin(chi)],
                        [Vgdot],
                        [(CTRL.gravity / Vg) * np.tan(state.phi) * np.cos(chi - psi)],
                        [0.0],
                        [0.0],
                        [psidot],
                        ])
        return xdot

    def h_gps(self, x, measurement, state):
        # measurement model for gps measurements
        pn = x.item(0)
        pe = x.item(1)
        Vg = x.item(2)
        chi = x.item(3)
        y = np.array([
            [pn],
            [pe],
            [Vg],
            [chi],
        ])
        return y

    def h_pseudo(self, x, measurement, state):
        # measurement model for wind triangale pseudo measurement
        pn = x.item(0)
        pe = x.item(1)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)
        y = np.array([
            [state.Va * np.cos(psi) + wn - Vg * np.cos(chi)],  # wind triangle x
            [state.Va * np.sin(psi) + we - Vg * np.sin(chi)],  # wind triangle y
        ])
        return y

    def propagate_model(self, measurement, state):
        # model propagation
        for i in range(0, self.N):
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
            # convert to discrete time models
            A_d = np.eye(7) + self.Ts * A + (self.Ts ** 2) * A @ A / 2.0
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + self.Ts**2 * self.Q

    def measurement_update(self, measurement, state):
        # always update based on wind triangle pseudo measurement
        yhat = self.h_pseudo(self.xhat, measurement, state)
        C = jacobian(self.h_pseudo, self.xhat, measurement, state)
        y = np.array([[0, 0]]).T
        S_inv = np.linalg.inv(self.R_pseudo + C @ self.P @ C.T)
        if True: #(y-h).T @ S_inv @ (y-h) < self.pseudo_threshold:
            L = self.P @ C.T @ S_inv
            tmp = np.eye(7) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R_pseudo @ L.T
            self.xhat = self.xhat + L @ (y - yhat)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            yhat = self.h_gps(self.xhat, measurement, state)
            C = jacobian(self.h_gps, self.xhat, measurement, state)
            y_chi = wrap(measurement.gps_course, yhat[3, 0])
            y = np.array([[measurement.gps_n,
                           measurement.gps_e,
                           measurement.gps_Vg,
                           y_chi]]).T
            S_inv = np.linalg.inv(self.R_gps + C @ self.P @ C.T)
            if True: #(y-h).T @ S_inv @ (y-h) < self.gps_threshold:
                L = self.P @ C.T @ S_inv
                self.xhat = self.xhat + L @ (y - yhat)
                tmp = np.eye(7) - L @ C
                self.P = tmp @ self.P @ tmp.T + L @ self.R_gps @ L.T

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course


def jacobian(fun, x, measurement, state):
    # compute jacobian of fun with respect to x
    f = fun(x, measurement, state)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.0001  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, measurement, state)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J
