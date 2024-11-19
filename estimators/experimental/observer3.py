"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/26/2020 - RWB
"""
import numpy as np
from scipy import stats
import parameters.control_parameters_airplane as CTRL
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
from tools.wrap import wrap
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors

class Observer:
    def __init__(self, ts, initial_measurements = MsgSensors()):
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


    def update(self, measurement):
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
        #self.estimated_state.alpha = self.estimated_state.theta
        self.estimated_state.beta = 0.0
        return self.estimated_state


class AlphaFilter:
    # alpha filter implements a simple low pass filter
    # y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u):
        self.y = self.alpha * self.y + (1-self.alpha) * u
        return self.y


class DirtyDerivative:
    # return the dirty derivative of signal y
    def __init__(self, Ts, sigma):
        beta = 0.9
        self.a1 = beta #(2.0 * sigma - Ts) / (2.0 * sigma + Ts)
        self.a2 = 1-beta #2.0 / (2.0 * sigma + Ts)
        self.y_dot = 0.0
        self.y_delay_1 = 0.0
        self.initialized = False

    def update(self, y):
        if self.initialized:
            self.y_dot = self.a1 * self.y_dot \
                        + self.a2 * (y - self.y_delay_1)
        else:
            self.y_dot = 0.
            self.initialized = True
        self.y_delay_1 = y
        return self.y_dot


class EkfAttitude:
    # implement continous-discrete EKF to estimate roll and pitch angles
    def __init__(self, ts):
        self.Q = 1e-6 * np.diag([1.0, 1.0, 100.0, 100.0, 1.0, 1.0, 1.0])
        self.xhat = np.array([[0.0], [0.0], [CTRL.Va0], [0.0], [0.0], [0.0], [0.0]])
        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.R_gyro = SENSOR.gyro_sigma**2 * np.diag([1.0, 1.0, 1.0])
        self.R_accel = SENSOR.accel_sigma**2 * np.diag([1.0, 1.0, 1.0])
        self.R = 1000*np.diag([(SENSOR.abs_pres_sigma/ CTRL.rho / CTRL.gravity)**2,
                               np.sqrt(2 * SENSOR.diff_pres_sigma**2 / CTRL.rho)])
        self.R = np.diag([1., 1.])
        self.N = 10  # number of prediction step per sample
        self.Ts = ts/self.N
        # initial state: phi, theta, u, w, bx, by, bz
        self.hdot = DirtyDerivative(SIM.ts_control, 0.05)
        self.gate_threshold = stats.chi2.isf(q=0.01, df=2)


    def update(self, measurement, state):
        self.propagate_model(measurement)
        self.measurement_update(measurement, state)
        state.phi = self.xhat.item(0)
        state.theta = self.xhat.item(1)
        u = self.xhat.item(2)
        w = self.xhat.item(3)
        state.bx = self.xhat.item(4)
        state.by = self.xhat.item(5)
        state.bz = self.xhat.item(6)
        state.alpha = -np.arctan(w/u)
        return state

    def f(self, x, measurement):
        # system dynamics for propagation model: xdot = f(x, u)
        phi = x.item(0)
        theta = x.item(1)
        u = x.item(2)
        w = x.item(3)
        bx = x.item(4)
        by = x.item(5)
        bz = x.item(6)
        p = measurement.gyro_x - bx
        q = measurement.gyro_y - by
        r = measurement.gyro_z - bz
        phidot = p + np.sin(phi) * np.tan(theta) * q\
                 + np.cos(phi) * np.tan(theta) * r
        thetadot = np.cos(phi) * q - np.sin(phi) * r
        udot = - w * q + measurement.accel_x
        wdot = u * q + CTRL.gravity + measurement.accel_z
        xdot = np.array([[phidot, thetadot, udot, wdot, 0.0, 0.0, 0.0]]).T
        return xdot


    def h(self, x, measurement):
        # measurement model y=h(x,u)
        phi = x.item(0)
        theta = x.item(1)
        u = x.item(2)
        w = x.item(3)
        Va = np.sqrt(u**2 + w**2)
        hdot = w * np.sin(theta) - w * np.cos(phi) * np.cos(theta)
        y = np.array([[Va, hdot]]).T
        return y

    def propagate_model(self, measurement):
        # model propagation
        for i in range(0, self.N):
            phi = self.xhat.item(0)
            theta = self.xhat.item(1)
            u = self.xhat.item(2)
            w = self.xhat.item(3)
            bx = self.xhat.item(4)
            by = self.xhat.item(5)
            bz = self.xhat.item(6)
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement)
            # compute G matrix for gyro noise
            G_gyro = np.array([[1., np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                               [0., np.cos(phi), -np.sin(phi)],
                               [0., -w, 0.],
                               [0., u, 0.],
                               [0., 0., 0.],
                               [0., 0., 0.],
                               [0., 0., 0.]])
            G_accel = np.array([[0., 0., 0.],
                                [0., 0., 0.],
                                [1., 0., 0.],
                                [0., 0., 1.],
                                [0., 0., 0.],
                                [0., 0., 0.],
                                [0., 0., 0.]])
            # convert to discrete time models
            A_d = np.eye(7) + self.Ts * A + (self.Ts ** 2) * A @ A
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T \
                     + self.Ts**2 * ( self.Q + G_gyro @ self.R_gyro @ G_gyro.T
                                      + G_accel @ self.R_accel @ G_accel.T )

    def measurement_update(self, measurement, state):
        # measurement updates
        C = jacobian(self.h, self.xhat, measurement)
        #Va = np.sqrt(2 * measurement.diff_pressure / CTRL.rho)
        Va = state.Va
        #hdot = self.hdot.update(measurement.abs_pressure / CTRL.rho / CTRL.gravity)
        hdot = self.hdot.update(state.altitude)
        y = np.array([[Va, hdot]]).T
        yhat = self.h(self.xhat, measurement)
        S_inv = np.linalg.inv(self.R + C @ self.P @ C.T)
        if True: #(y-h).T @ S_inv @ (y-h) < self.gate_threshold:
            L = self.P @ C.T @ S_inv
            tmp = np.eye(7) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R @ L.T
            self.xhat = self.xhat + L @ (y - yhat)
            print('updated')


class EkfPosition:
    # implement continous-discrete EKF to estimate pn, pe, chi, Vg, wn, we, psi
    def __init__(self, ts):
        self.Q = np.diag([
                    0.1,  # pn
                    0.1,  # pe
                    0.1,  # Vg
                    0.0001, # chi
                    0.1, # wn
                    0.1, # we
                    0.001, # psi
                    ])
        self.xhat = np.array([[0.0], [0.0], [25.0], [0.0], [0.0], [0.0], [0.0]])
        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.R_gps = np.diag([
                    SENSOR.gps_n_sigma**2,  # y_gps_n
                    SENSOR.gps_e_sigma**2,  # y_gps_e
                    SENSOR.gps_Vg_sigma**2,  # y_gps_Vg
                    SENSOR.gps_course_sigma**2,  # y_gps_course
                    ])
        self.R_pseudo = np.diag([
                    0.000001,  # pseudo measurement #1
                    0.000001,  # pseudo measurement #2
                    ])
        self.N = 10  # number of prediction step per sample
        self.Ts = ts / self.N
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.pn = self.xhat.item(0)
        state.pe = self.xhat.item(1)
        state.Vg = self.xhat.item(2)
        state.chi = self.xhat.item(3)
        state.wn = self.xhat.item(4)
        state.we = self.xhat.item(5)
        state.psi = self.xhat.item(6)

    def f(self, x, state):
        # system dynamics for propagation model: xdot = f(x, u)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)
        psidot = (state.q * np.sin(state.phi) + state.r * np.cos(state.phi)) / np.cos(state.theta)
        Vgdot = ((state.Va * np.cos(psi) + wn) * (-psidot * state.Va * np.sin(psi))
                 + (state.Va * np.sin(psi) + we) * (psidot * state.Va * np.cos(psi))) / Vg
        xdot = np.array([[Vg * np.cos(chi)],
                       [Vg * np.sin(chi)],
                       [Vgdot],
                       [(CTRL.gravity / Vg) * np.tan(state.phi) * np.cos(chi - psi)],
                       [0.0],
                       [0.0],
                       [psidot],
                       ])
        return xdot

    def h_gps(self, x, state):
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

    def h_pseudo(self, x, state):
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
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, state)
            # update P with continuous time model
            # self.P = self.P + self.Ts * (A @ self.P + self.P @ A.T + self.Q + G @ self.Q_gyro @ G.T)
            # convert to discrete time models
            A_d = np.eye(7) + self.Ts * A + (self.Ts ** 2) * A @ A
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + self.Ts**2 * self.Q

    def measurement_update(self, measurement, state):
        # always update based on wind triangle pseudu measurement
        yhat = self.h_pseudo(self.xhat, state)
        C = jacobian(self.h_pseudo, self.xhat, state)
        y = np.array([[0, 0]]).T
        L = self.P @ C.T @ np.linalg.inv(self.R_pseudo + C @ self.P @ C.T)
        tmp = np.eye(7) - L @ C
        self.P = tmp @ self.P @ tmp.T + L @ self.R_pseudo @ L.T
        self.xhat = self.xhat + L @ (y - yhat)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            yhat = self.h_gps(self.xhat, state)
            C = jacobian(self.h_gps, self.xhat, state)
            y_chi = wrap(measurement.gps_course, yhat[3, 0])
            y = np.array([[measurement.gps_n,
                           measurement.gps_e,
                           measurement.gps_Vg,
                           y_chi]]).T
            L = self.P @ C.T @ np.linalg.inv(self.R_gps + C @ self.P @ C.T)
            self.xhat = self.xhat + L @ (y - yhat)
            tmp = np.eye(7) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R_gps @ L.T

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course

def jacobian(fun, x, measurement):
    # compute jacobian of fun with respect to x
    f = fun(x, measurement)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.0001  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, measurement)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J