"""
quadrotor_dynamics
    - This file builds on chap4.quadrotor_dynamics to add sensor outputs

part of quadSim
    - Beard & McLain, PUP, 2012
    - Update history:
        5/2/2022 - RWB
        7/13/2023 - RWB
"""
import numpy as np
from models.quadrotor_dynamics import QuadrotorDynamics as QuadrotorDynamicsNoSensors
from tools.rotations import quaternion_to_rotation, euler_to_rotation, quaternion_to_euler
from message_types.msg_sensors import MsgSensors
import parameters.quadrotor_parameters as QUAD
import parameters.sensor_parameters as SENSOR
from tools.wrap import wrap


class QuadrotorDynamics(QuadrotorDynamicsNoSensors):
    def __init__(self, Ts):
        super().__init__(Ts)
        # initialize the sensors message
        self._sensors = MsgSensors()
        self.psi = 0.
        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.

    ###################################
    # public functions
    def sensors(self):
        "Return value of sensors on MAV: gyros, accels, absolute_pressure, dynamic_pressure, GPS"
        R_b_i = quaternion_to_rotation(self._state[6:10])  # body to inertial
        vel = self._state[3:6]
        # simulate rate gyros(units are rad / sec)
        self._sensors.gyro_x = self._state.item(10)\
                               + np.random.normal(SENSOR.gyro_x_bias, SENSOR.gyro_sigma)
        self._sensors.gyro_y = self._state.item(11) \
                               + np.random.normal(SENSOR.gyro_y_bias, SENSOR.gyro_sigma)
        self._sensors.gyro_z = self._state.item(12) \
                               + np.random.normal(SENSOR.gyro_z_bias, SENSOR.gyro_sigma)
        # simulate accelerometers(units of g)
        e3 = np.array([[0.0], [0.0], [1.0]])
        specific_acceleration = R_b_i.T @ (self._forces/QUAD.mass - QUAD.gravity * e3)
        self._sensors.accel_x = specific_acceleration.item(0) \
                                + np.random.normal(0., SENSOR.accel_sigma)
        self._sensors.accel_y = specific_acceleration.item(1) \
                                + np.random.normal(0., SENSOR.accel_sigma)
        self._sensors.accel_z = specific_acceleration.item(2) \
                                + np.random.normal(0., SENSOR.accel_sigma)
        # simulate magnetometers
        # magnetic field in provo has magnetic declination of 12.5 degrees
        # and magnetic inclination of 66 degrees
        Rot_mag = euler_to_rotation(0.0, np.radians(-66), np.radians(12.5))
        # magnetic field in inertial frame: unit vector
        mag_inertial = Rot_mag.T @ np.array([[1.0], [0.0], [0.0]])
        # magnetic field in body frame: unit vector
        mag_body = R_b_i.T @ mag_inertial
        self._sensors.mag_x = mag_body.item(0) + np.random.normal(0., SENSOR.mag_sigma)
        self._sensors.mag_y = mag_body.item(1) + np.random.normal(0., SENSOR.mag_sigma)
        self._sensors.mag_z = mag_body.item(2) + np.random.normal(0., SENSOR.mag_sigma)
        # simulate digital compass
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        self.psi = wrap(psi, self.psi)
        self._sensors.compass = self.psi + np.random.normal(0., SENSOR.mag_sigma)
        # simulate pressure sensors
        self._sensors.abs_pressure = -QUAD.rho * QUAD.gravity * self._state.item(2) \
                            + np.random.normal(0., SENSOR.abs_pres_sigma)
        # simulate GPS sensor
        if self._t_gps >= SENSOR.ts_gps:
            self._gps_eta_n = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_n \
                             + np.random.normal(0., SENSOR.gps_n_sigma)
            self._gps_eta_e = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_e \
                             + np.random.normal(0., SENSOR.gps_e_sigma)
            self._gps_eta_h = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_h \
                             + np.random.normal(0., SENSOR.gps_h_sigma)
            self._sensors.gps_n = self._state.item(0) # + self._gps_eta_n
            self._sensors.gps_e = self._state.item(1) # + self._gps_eta_e
            self._sensors.gps_h = -self._state.item(2) # + self._gps_eta_h
            self._sensors.gps_Vg = np.linalg.norm(vel) \
                                           + np.random.normal(0., SENSOR.gps_Vg_sigma)
            self._sensors.gps_course = np.arctan2(vel.item(1), vel.item(0)) \
                                      + np.random.normal(0., SENSOR.gps_course_sigma)
            self._t_gps = 0.
        else:
            self._t_gps += self.ts_simulation
        return self._sensors

