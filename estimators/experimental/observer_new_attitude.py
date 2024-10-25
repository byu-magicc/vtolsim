"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
        2/28/2024 - RWB
    - attempt to use velocity and attitude dynamics to better estimate roll, pitch, and biases
    - measurements are airspeed, and pseudo measurement setting sideslip velocity v to zero
"""
import numpy as np
from scipy import stats
import parameters.control_parameters as CTRL
import parameters.sensor_parameters as SENSOR
from tools.wrap import wrap
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors

class Observer:
    def __init__(self, ts_control, initial_measurements = MsgSensors()):
        # initialized estimated state message
        self.ts = ts_control
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
        self.attitude_ekf = EkfAttitude(self.ts)
        # ekf for pn, pe, Vg, chi, wn, we, psi
        self.position_ekf = EkfPosition(self.ts)

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

        # estimate u, v, w, phi, theta, bx, by, bz with ekf
        self.attitude_ekf.update(measurement, self.estimated_state)

        # estimate pn, pe, Vg, chi, wn, we, psi
        self.position_ekf.update(measurement, self.estimated_state)

        # not estimating these
        self.estimated_state.alpha = self.estimated_state.theta
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


class EkfAttitude:
    # implement continous-discrete EKF to estimate u, v, w, phi, theta, bx, by, bz
    def __init__(self, ts):
        self.ts = ts
        self.Q = 1e-8 * np.diag([1.0, 1.0, 1.0, 10.0, 10.0, 1.0, 1.0, 1.0])
        self.P = np.diag([10.0, 1.0, 10.0, 1.0, 1.0, 10.0, 10.0, 10.0])
        self.R_gyro = SENSOR.gyro_sigma**2 * np.diag([1.0, 1.0, 1.0])
        self.R_accel = SENSOR.accel_sigma ** 2 * np.diag([1.0, 1.0, 1.0])
        self.R = np.diag([0.01, .00001])  # measurement noise (Va and v)
        self.xhat = np.array([
            [CTRL.Va0], # u (0)
            [0.], # v (1)
            [0.0], # w (2)
            [0.0], # phi  (3)
            [0.0], # theta (4)
            [0.0], # bx (5)
            [0.0], # by (6)
            [0.0], # bz (7)
            ]) 
        self.N = 10  # number of prediction step per sample
        self.Ts = self.ts/self.N
        self.gate_threshold = stats.chi2.isf(q=0.1, df=2)

    def update(self, measurement, state):
        self.propagate_model(measurement)
        self.measurement_update(measurement, state.Va)
        state.phi = self.xhat.item(3)
        state.theta = self.xhat.item(4)
        state.bx = self.xhat.item(5)
        state.by = self.xhat.item(6)
        state.bz = self.xhat.item(7)

    def f(self, x, measurement):
        # system dynamics for propagation model: xdot = f(x, u)
        u = x.item(0)
        v = x.item(1)
        w = x.item(2)
        phi = x.item(3)
        theta = x.item(4)
        p = measurement.gyro_x - x.item(5)
        q = measurement.gyro_y - x.item(6)
        r = measurement.gyro_z - x.item(7)
        udot = r * v - q * w - CTRL.gravity * np.sin(theta) + measurement.accel_x
        vdot = p * w - r * u + CTRL.gravity * np.cos(theta) * np.sin(phi) + measurement.accel_y
        wdot = q * u - p * v + CTRL.gravity * np.cos(theta) * np.cos(phi) + measurement.accel_z
        phidot = p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
        thetadot = q * np.cos(phi) - r * np.sin(phi)
        bxdot = 0
        bydot = 0 
        bzdot = 0
        xdot = np.array([[udot], [vdot], [wdot], [phidot], [thetadot], [bxdot], [bydot], [bzdot]])
        return xdot

    def h(self, x, measurement):
        # measurement model: y = h(x, u)
        u = x.item(0)
        v = x.item(1)
        w = x.item(2)
        # phi = x.item(3)
        # theta = x.item(4)
        # p = measurement.gyro_x - x.item(5)
        # q = measurement.gyro_y - x.item(6)
        # r = measurement.gyro_z - x.item(7)
         # measurement model y
        y_Va = np.sqrt(u**2 + w**2)
        y_v = v
        return np.array([[y_Va], [y_v]])

    def propagate_model(self, measurement):
        # model propagation
        Tp = self.Ts
        for i in range(0, self.N):
            u = self.xhat.item(0)
            v = self.xhat.item(1)
            w = self.xhat.item(2)
            phi = self.xhat.item(3)
            theta = self.xhat.item(4)
            # propagate model
            self.xhat = self.xhat + Tp * self.f(self.xhat, measurement)
            # compute Jacobian
            A = self.jacobian(self.f, self.xhat, measurement)
            # compute G matrix for gyro and accel noise
            Gg = -np.array([
                [0, w, -v],
                [-w, 0, u],
                [v, -u, 0],
                [-1, -np.sin(phi) * np.tan(theta), -np.cos(phi) * np.tan(theta)],
                [0.0, -np.cos(phi), np.sin(phi)],
                [0., 0., 0.],
                [0., 0., 0.],
                [0., 0., 0.],
                ])
            Ga = np.concatenate((-np.eye(3), np.zeros((5,3))), axis=0)
            # convert to discrete time models
            A_d = np.eye(8) + Tp * A + ((Tp ** 2)/2.) * A @ A
            # update P with discrete time model
            Qeff = self.Q + Gg @ self.R_gyro @ Gg.T + Ga @ self.R_accel @ Ga.T
            self.P = A_d @ self.P @ A_d.T + Tp**2 * Qeff

    def measurement_update(self, measurement, Va):
        # measurement updates
        h = self.h(self.xhat, measurement)
        C = self.jacobian(self.h, self.xhat, measurement)
        y = np.array([
            [Va], 
            [0.0], # pseudo measurement of v
            ])
        S_inv = np.linalg.inv(self.R + C @ self.P @ C.T)
        if True: #(y-h).T @ S_inv @ (y-h) < self.gate_threshold:
            L = self.P @ C.T @ S_inv
            tmp = np.eye(8) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R @ L.T
            self.xhat = self.xhat + L @ (y - h)
            # print('updating')
    
    def jacobian(self, fun, x, measurement):
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


class EkfPosition:
    # implement continous-discrete EKF to estimate pn, pe, Vg, chi, wn, we, psi
    def __init__(self, ts):
        self.ts = ts
        self.Q = np.diag([
                    0.1,  # pn
                    0.1,  # pe
                    0.1,  # Vg
                    0.1, # chi
                    0.1, # wn
                    0.1, # we
                    0.1, #0.0001, # psi
                    ])
        self.R_gps = np.diag([
                    SENSOR.gps_n_sigma**2,  # y_gps_n
                    SENSOR.gps_e_sigma**2,  # y_gps_e
                    SENSOR.gps_Vg_sigma**2,  # y_gps_Vg
                    SENSOR.gps_course_sigma**2,  # y_gps_course
                    ])
        self.R_pseudo = np.diag([
                    0.01,  # pseudo measurement #1
                    0.01,  # pseudo measurement #2
                    ])
        self.N = 10  # number of prediction step per sample
        self.Ts = (self.ts / self.N)
        self.xhat = np.array([[0.0], [0.0], [25.0], [0.0], [0.0], [0.0], [0.0]])
        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
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
        Va = np.sqrt(2 * measurement.diff_pressure / CTRL.rho)
        q = measurement.gyro_y
        r = measurement.gyro_z
        # psidot = (state.q * np.sin(state.phi) + state.r * np.cos(state.phi)) / np.cos(state.theta)
        # Vgdot = ((state.Va * np.cos(psi) + wn) * (-psidot * state.Va * np.sin(psi))
        #          + (state.Va * np.sin(psi) + we) * (psidot * state.Va * np.cos(psi))) / Vg
        psidot = (q * np.sin(state.phi) + r * np.cos(state.phi)) / np.cos(state.theta)
        Vgdot = ((Va* np.cos(psi) + wn) * (-psidot * Va * np.sin(psi))
                 + (Va * np.sin(psi) + we) * (psidot * Va * np.cos(psi))) / Vg
        f_ = np.array([[Vg * np.cos(chi)],
                       [Vg * np.sin(chi)],
                       [Vgdot],
                       [(CTRL.gravity / Vg) * np.tan(state.phi) * np.cos(chi - psi)],
                       [0.0],
                       [0.0],
                       [psidot],
                       ])
        return f_

    def h_gps(self, x, measurement, state):
        # measurement model for gps measurements
        pn = x.item(0)
        pe = x.item(1)
        Vg = x.item(2)
        chi = x.item(3)
        h_ = np.array([
            [pn],
            [pe],
            [Vg],
            [chi],
        ])
        return h_

    def h_pseudo(self, x, measurement, state):
        # measurement model for wind triangale pseudo measurement
        pn = x.item(0)
        pe = x.item(1)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)
        h_ = np.array([
            [state.Va * np.cos(psi) + wn - Vg * np.cos(chi)],  # wind triangle x
            [state.Va * np.sin(psi) + we - Vg * np.sin(chi)],  # wind triangle y
        ])
        return h_

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
        h = self.h_pseudo(self.xhat, measurement, state)
        C = jacobian(self.h_pseudo, self.xhat, measurement, state)
        y = np.array([[0, 0]]).T
        S_inv = np.linalg.inv(self.R_pseudo + C @ self.P @ C.T)
        if (y-h).T @ S_inv @ (y-h) < self.pseudo_threshold:
            L = self.P @ C.T @ S_inv
            tmp = np.eye(7) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R_pseudo @ L.T
            self.xhat = self.xhat + L @ (y - h)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = self.h_gps(self.xhat, measurement, state)
            C = jacobian(self.h_gps, self.xhat, measurement, state)
            y_chi = wrap(measurement.gps_course, h[3, 0])
            y = np.array([[measurement.gps_n,
                           measurement.gps_e,
                           measurement.gps_Vg,
                           y_chi]]).T
            S_inv = np.linalg.inv(self.R_gps + C @ self.P @ C.T)
            if (y-h).T @ S_inv @ (y-h) < self.gps_threshold:
                L = self.P @ C.T @ S_inv
                self.xhat = self.xhat + L @ (y - h)
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
