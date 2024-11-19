"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/4/2019 - RWB
        3/6/2024 - RWB
"""
import numpy as np
from scipy import stats
import parameters.control_parameters_airplane as CTRL
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
import parameters.anaconda_parameters as QUAD
from tools.rotations import euler_to_rotation
from tools.wrap import wrap
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from estimators.filters import AlphaFilter

class Observer:
    def __init__(self, ts):
        # initialized estimated state message
        self.ekf = EkfFullState(ts)

    def update(self, measurement):
        estimated_state = self.ekf.update(measurement)
        return estimated_state


class EkfFullState:
    # implement continous-discrete EKF to estimate full state
    def __init__(self, ts, initial_measurements = MsgSensors()):
        self.Q = np.diag([
                        (0.000003)**2,  # pn
                        (0.000003)**2,  # pe
                        (0.000003)**2,  # pd
                        (0.0001)**2,  # u
                        (0.01)**2,  # v
                        (0.0001)**2,  # w
                        (0.0000001)**2,  # phi
                        (0.0000001)**2,  # theta
                        (0.0000001)**2,  # psi
                        (0.0000001)**2,  # bx
                        (0.0000001)**2,  # by
                        (0.0000001)**2,  # bz
                        (0.0003)**2,  # wn
                        (0.0003)**2,  # we
                        ])
        self.P = 0.001*np.diag([
                    10**2,  # pn
                    10**2,  # pe
                    10**2,  # pd
                    2**2,  # u
                    2**2,  # v
                    2**2,  # w
                    np.radians(35)**2,  # phi
                    np.radians(35)**2,  # theta
                    np.radians(45)**2,  # psi
                    np.radians(10)**2,  # bx
                    np.radians(10)**2,  # by
                    np.radians(10)**2,  # bz
                    15**2,  # wn
                    15**2,  # we
                   ])
        self.xhat = np.array([[
                    QUAD.north0,  # pn
                    QUAD.east0,  # pe
                    QUAD.down0,  # pd
                    QUAD.Va0,  # u
                    0,  # v
                    0,  # w
                    0,  # phi
                    0,  # theta
                    QUAD.psi0,  # psi
                    0,  # bx
                    0,  # by
                    0,  # bz
                    0,  # wn
                    0,  # we
                  ]]).T
        self.Q_gyro = SENSOR.gyro_sigma**2 * np.eye(3)
        self.Q_accel = SENSOR.accel_sigma**2 * np.eye(3)
        self.R_analog = np.diag([
            SENSOR.abs_pres_sigma**2,
            SENSOR.diff_pres_sigma**2,
            (0.01)**2
        ])
        self.R_gps = np.diag([
            SENSOR.gps_n_sigma**2,
            SENSOR.gps_e_sigma**2,
            SENSOR.gps_Vg_sigma**2,
            SENSOR.gps_course_sigma**2
        ])
        self.R_pseudo = np.diag([
                    (0.1)**2,  # pseudo measurement #1
                    (0.1)**2,  # pseudo measurement #2
                    ])
        self.N = 10  # number of prediction step per sample
        self.Ts = ts / self.N
        self.analog_threshold = stats.chi2.isf(q=0.01, df=3)
        self.pseudo_threshold = stats.chi2.isf(q=0.01, df=2)
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999
        self.estimated_state = MsgState()
        self.elapsed_time = 0
        self.lpf_gyro_x = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_x)
        self.lpf_gyro_y = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_y)
        self.lpf_gyro_z = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_z)


    def update(self, measurement):
        self.propagate_model(measurement)
        self.measurement_update(measurement)
        # write out estimate state
        self.estimated_state.north = self.xhat.item(0)
        self.estimated_state.east = self.xhat.item(1)
        self.estimated_state.altitude = -self.xhat.item(2)
        vel_body = self.xhat[3:6]
        self.estimated_state.phi = self.xhat.item(6)
        self.estimated_state.theta = self.xhat.item(7)
        self.estimated_state.psi = self.xhat.item(8)
        self.estimated_state.bx = self.xhat.item(9)
        self.estimated_state.by = self.xhat.item(10)
        self.estimated_state.bz = self.xhat.item(11)
        self.estimated_state.wn = self.xhat.item(12)
        self.estimated_state.we = self.xhat.item(13)
        # estimate needed quantities that are not part of state
        R = euler_to_rotation(
            self.estimated_state.phi,
            self.estimated_state.theta,
            self.estimated_state.psi)
        vel_world = R @ vel_body
        wind_world = np.array([[self.estimated_state.wn], [self.estimated_state.we], [0]])
        wind_body = R.T @ wind_world
        vel_rel = vel_body - wind_body
        self.estimated_state.Va = np.linalg.norm(vel_rel)
        self.estimated_state.alpha = np.arctan(vel_rel.item(2) / vel_rel.item(0))
        self.estimated_state.beta = np.arcsin(vel_rel.item(1) / self.estimated_state.Va)
        self.estimated_state.Vg = np.linalg.norm(vel_world)
        self.estimated_state.chi = np.arctan2(vel_world.item(1), vel_world.item(0))
        self.estimated_state.p = self.lpf_gyro_x.update(measurement.gyro_x) - self.estimated_state.bx
        self.estimated_state.q = self.lpf_gyro_y.update(measurement.gyro_y) - self.estimated_state.by
        self.estimated_state.r = self.lpf_gyro_z.update(measurement.gyro_z) - self.estimated_state.bz
        self.elapsed_time += SIM.ts_control
        return self.estimated_state

    def f(self, x, measurement):
        # system dynamics for propagation model: xdot = f(x, u)
        # pos   = x[0:3]
        vel = x[3:6]
        Theta = x[6:9]
        bias = x[9:12]
        # wind = np.array([[x.item(12), x.item(13), 0]]).T
        y_gyro = np.array([[measurement.gyro_x, measurement.gyro_y, measurement.gyro_z]]).T
        y_accel = np.array([[measurement.accel_x, measurement.accel_y, measurement.accel_z]]).T
        R = euler_to_rotation(Theta.item(0), Theta.item(1), Theta.item(2))
        omega = y_gyro - bias
        pos_dot = R @ vel
        vel_dot = cross(vel) @ omega + y_accel + R.T @ np.array([[0, 0, CTRL.gravity]]).T
        Theta_dot = S(Theta) @ omega
        bias_dot = np.array([[0, 0, 0]]).T
        wind_dot = np.array([[0, 0]]).T
        xdot = np.concatenate((pos_dot, vel_dot, Theta_dot, bias_dot, wind_dot), axis=0)
        return xdot

    def h_analog(self, x, measurement):
        # analog sensor measurements and pseudo measurements
        pos = x[0:3]
        vel_body = x[3:6]
        Theta = x[6:9]
        #bias = x[9:12]
        wind_world = np.array([[x.item(12), x.item(13), 0]]).T
        R = euler_to_rotation(Theta.item(0), Theta.item(1), Theta.item(2))
        wind_body = R.T @ wind_world
        vel_rel = vel_body - wind_body
        Va = np.linalg.norm(vel_rel)
        abs_pres = -CTRL.rho * CTRL.gravity * pos.item(2)
        diff_pres = 0.5 * CTRL.rho * (Va**2)
        sideslip = vel_rel.item(1)
        y = np.array([[abs_pres, diff_pres, sideslip]]).T
        return y

    def h_gps(self, x, measurement):
        # measurement model for gps measurements
        pos = x[0:3]
        vel_body = x[3:6]
        Theta = x[6:9]
        R = euler_to_rotation(Theta.item(0), Theta.item(1), Theta.item(2))
        vel_world = R @ vel_body
        pn = pos.item(0)
        pe = pos.item(1)
        Vg = np.linalg.norm(vel_world)
        chi = np.arctan2(vel_world.item(1), vel_world.item(0))
        y = np.array([[pn, pe, Vg, chi]]).T
        return y

    def h_pseudo(self, x, measurement):
        # measurement model for wind triangale pseudo measurement
        #pos = x[0:3]
        vel_body = x[3:6]
        Theta = x[6:9]
        #bias = x[9:12]
        wind_world = np.array([[x.item(12), x.item(13), 0]]).T
        R = euler_to_rotation(Theta.item(0), Theta.item(1), Theta.item(2))
        wind_body = R.T @ wind_world
        vel_rel = vel_body - wind_body
        vel_world = R @ vel_body
        Va = np.linalg.norm(vel_rel)
        Vg = np.linalg.norm(vel_world)
        chi = np.arctan2(vel_world.item(1), vel_world.item(0))
        wn = wind_world.item(0)
        we = wind_world.item(1)
        psi = Theta.item(2)
        y = np.array([
            [Va * np.cos(psi) + wn - Vg * np.cos(chi)],  # wind triangle x
            [Va * np.sin(psi) + we - Vg * np.sin(chi)],  # wind triangle y
        ])
        return y

    def propagate_model(self, measurement):
        # model propagation
        for i in range(0, self.N):
            vel = self.xhat[3:6]
            Theta = self.xhat[6:9]
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement)
            # convert to discrete time models
            A_d = np.eye(14) + self.Ts * A + (self.Ts ** 2) * A @ A / 2.0
            Gg_d =  np.concatenate((np.zeros((3, 3)), -cross(vel), -S(Theta), np.zeros((5, 3))), axis=0)
            Ga_d = np.concatenate((np.zeros((3, 3)), -np.eye(3), np.zeros((8, 3))), axis=0)
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + self.Ts**2 *  ( \
                          self.Q\
                        + Gg_d @ self.Q_gyro @ Gg_d.T\
                        + Ga_d @ self.Q_accel @ Ga_d.T )

    def measurement_update(self, measurement):
        # always update based on sensor measurements
        yhat = self.h_analog(self.xhat, measurement)
        C = jacobian(self.h_analog, self.xhat, measurement)
        y = np.array([[measurement.abs_pressure,
                      measurement.diff_pressure,
                      0.0, # sideslip
                      ]]).T
        S_inv = np.linalg.inv(self.R_analog + C @ self.P @ C.T)
        if True:  #(y - h).T @ S_inv @ (y - h) < self.analog_threshold:
            L = self.P @ C.T @ S_inv
            tmp = np.eye(14) - L @ C
            self.P = tmp @ self.P @ tmp.T +  L @ self.R_analog @ L.T
            self.xhat = self.xhat + L @ (y - yhat)

        # always update based on wind triangle pseudo measurement
        yhat = self.h_pseudo(self.xhat, measurement)
        C = jacobian(self.h_pseudo, self.xhat, measurement)
        y = np.array([[0, 0]]).T
        S_inv = np.linalg.inv(self.R_pseudo + C @ self.P @ C.T)
        if (self.elapsed_time>1):  #(y-yhat).T @ S_inv @ (y-yhat) < self.pseudo_threshold:
            L = self.P @ C.T @ S_inv
            tmp = np.eye(14) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R_pseudo @ L.T
            self.xhat = self.xhat + L @ (y - yhat)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            yhat = self.h_gps(self.xhat, measurement)
            C = jacobian(self.h_gps, self.xhat, measurement)
            y_chi = wrap(measurement.gps_course, yhat.item(3))
            y = np.array([[measurement.gps_n, measurement.gps_e, measurement.gps_Vg, y_chi]]).T
            if (self.elapsed_time>0.5): #np.linalg.norm(y-h)<0.5:
                L = self.P @ C.T @ np.linalg.inv(self.R_gps + C @ self.P @ C.T)
                tmp = np.eye(14) - L @ C
                self.P = tmp @ self.P @ tmp.T + L @ self.R_gps @ L.T
                self.xhat = self.xhat + L @ (y - yhat)

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course


def cross(vec):
    return np.array([[0, -vec.item(2), vec.item(1)],
                     [vec.item(2), 0, -vec.item(0)],
                     [-vec.item(1), vec.item(0), 0]])


def S(Theta):
    return np.array([[1,
                      np.sin(Theta.item(0)) * np.tan(Theta.item(1)),
                      np.cos(Theta.item(0)) * np.tan(Theta.item(1))],
                     [0,
                      np.cos(Theta.item(0)),
                      -np.sin(Theta.item(0))],
                     [0,
                      (np.sin(Theta.item(0)) / np.cos(Theta.item(1))),
                      (np.cos(Theta.item(0)) / np.cos(Theta.item(1)))]
                     ])


#def jacobian(fun, x, measurement):
def jacobian(fun, x, args=[]):
    # compute jacobian of fun with respect to x
    f = fun(x, args)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.01  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, args)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J