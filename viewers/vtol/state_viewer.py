"""
data_viewer

part of mavsimPy
    - Beard & McLain, PUP, 2012
    - Update history:
        12/17/2018 - RWB
        1/14/2019 - RWB
"""
import numpy as np
from viewers.plotter import Plotter
from tools.wrap import wrap
from tools.rotations import quaternion_to_euler

class dataViewer:
    def __init__(self, app,  dt = 0.01,
                 time_window_length = 30, # number of data points plotted at a time
                 plot_period = 0.2, # time interval between a plot update
                 data_recording_period = 0.1): # time interval between recording a data update
        self._dt = dt
        self._data_window_length= time_window_length/data_recording_period
        self._update_counter = 0
        self._plots_per_row = 3
        self._plotter = Plotter(app=app, plots_per_row=self._plots_per_row)  # plot last time_window seconds of data
        self._plot_period = plot_period
        self._data_recording_period = data_recording_period
        self._plot_delay = 0
        self._data_recording_delay = 0
        self._time = 0

        #define colors
        truth_color = (0,255,0)
        truth_color_2 = (160,202,111)
        truth_color_3 = (124,230,167)
        estimate_color = (255,0,0)
        estimate_color_2 = (255,150,150)
        estimate_color_3 = (255,154,111)
        command_color = (0,0,255)

        # define first row
        self._plotter.create_plot_widget(plot_id='north', xlabel='Time (s)', ylabel='north (m)',
                                        window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='east', xlabel='Time (s)', ylabel='east (m)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='altitude', xlabel='Time (s)', ylabel='altitude (m)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="north", data_label="north", data_color=truth_color)
        self._plotter.create_data_set(plot_id="north", data_label="north_e", data_color=estimate_color) 
        self._plotter.create_data_set(plot_id="north", data_label="north_c", data_color=command_color) 
        self._plotter.create_data_set(plot_id="east", data_label="east", data_color=truth_color)
        self._plotter.create_data_set(plot_id="east", data_label="east_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="east", data_label="east_c", data_color=command_color)
        self._plotter.create_data_set(plot_id="altitude", data_label="altitude", data_color=truth_color)
        self._plotter.create_data_set(plot_id="altitude", data_label="altitude_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="altitude", data_label="altitude_c", data_color=command_color)

        # define second row
        self._plotter.create_plot_widget(plot_id='vel_n', xlabel='Time (s)', ylabel='vel_n (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='vel_e', xlabel='Time (s)', ylabel='vel_e (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='vel_d', xlabel='Time (s)', ylabel='vel_d (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="vel_n", data_label="vel_n", data_color=truth_color)
        self._plotter.create_data_set(plot_id="vel_n", data_label="vel_n_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="vel_n", data_label="vel_n_c", data_color=command_color)
        self._plotter.create_data_set(plot_id="vel_e", data_label="vel_e", data_color=truth_color)
        self._plotter.create_data_set(plot_id="vel_e", data_label="vel_e_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="vel_e", data_label="vel_e_c", data_color=command_color)
        self._plotter.create_data_set(plot_id="vel_d", data_label="vel_d", data_color=truth_color)
        self._plotter.create_data_set(plot_id="vel_d", data_label="vel_d_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="vel_d", data_label="vel_d_c", data_color=command_color)

       # define third row
        self._plotter.create_plot_widget(plot_id='roll', xlabel='Time (s)', ylabel='roll (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='pitch', xlabel='Time (s)', ylabel='pitch (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='yaw', xlabel='Time (s)', ylabel='yaw (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="roll", data_label="roll", data_color=truth_color)
        self._plotter.create_data_set(plot_id="roll", data_label="roll_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="roll", data_label="roll_c", data_color=command_color)
        self._plotter.create_data_set(plot_id="pitch", data_label="pitch", data_color=truth_color)
        self._plotter.create_data_set(plot_id="pitch", data_label="pitch_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="pitch", data_label="pitch_c", data_color=command_color)
        self._plotter.create_data_set(plot_id="yaw", data_label="yaw", data_color=truth_color)
        self._plotter.create_data_set(plot_id="yaw", data_label="yaw_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="yaw", data_label="yaw_c", data_color=command_color)

        # define fourth row
        self._plotter.create_plot_widget(plot_id='omega_x', xlabel='Time (s)', ylabel='omega_x (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='omega_y', xlabel='Time (s)', ylabel='omega_y (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='omega_z', xlabel='Time (s)', ylabel='omega_z (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="omega_x", data_label="omega_x", data_color=truth_color)
        self._plotter.create_data_set(plot_id="omega_x", data_label="omega_x_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="omega_y", data_label="omega_y", data_color=truth_color)
        self._plotter.create_data_set(plot_id="omega_y", data_label="omega_y_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="omega_z", data_label="omega_z", data_color=truth_color)
        self._plotter.create_data_set(plot_id="omega_z", data_label="omega_z_e", data_color=estimate_color)

        self._plotter.show_window()

    def update(self, true_state, estimated_state, commanded_state, delta):
        if self._data_recording_delay >= self._data_recording_period:
            self.__update_data(true_state, estimated_state, commanded_state, delta, self._time)
            self._data_recording_delay = 0
        if self._plot_delay >= self._plot_period:
            self.__update_plot()
            self._plot_delay = 0
        self._plot_delay += self._dt
        self._data_recording_delay += self._dt
        self._time += self._dt

    def __update_data(self, true_state, estimated_state, commanded_state, delta, t):
        #add the true state data
        if true_state != None:
            # phi, theta, psi = rotation_to_euler(true_state.R)
            phi, theta, psi = quaternion_to_euler(true_state[6:10])
            true_st_euler = np.concatenate((true_state[0:6], np.array([[phi, theta, psi]]).T, true_state[10:13]))
            true_st_euler = np.squeeze(true_st_euler)
            self._plotter.add_data_point(plot_id='north', data_label='north', xvalue=t, yvalue=true_state.pos[0,0])
            self._plotter.add_data_point(plot_id='east', data_label='east', xvalue=t, yvalue=true_state.pos[1,0])
            self._plotter.add_data_point(plot_id='altitude', data_label='altitude', xvalue=t, yvalue=-true_state.pos[2,0])
            self._plotter.add_data_point(plot_id='vel_n', data_label='vel_n', xvalue=t, yvalue=true_state.vel[0,0])
            self._plotter.add_data_point(plot_id='vel_e', data_label='vel_e', xvalue=t, yvalue=true_state.vel[1,0])
            self._plotter.add_data_point(plot_id='vel_d', data_label='vel_d', xvalue=t, yvalue=true_state.vel[2,0])
            self._plotter.add_data_point(plot_id='roll', data_label='roll', xvalue=t, yvalue=self.__rad_to_deg(phi))
            self._plotter.add_data_point(plot_id='pitch', data_label='pitch', xvalue=t, yvalue=self.__rad_to_deg(theta))
            self._plotter.add_data_point(plot_id='yaw', data_label='yaw', xvalue=t, yvalue=self.__rad_to_deg(psi))
            self._plotter.add_data_point(plot_id='omega_x', data_label='omega_x', xvalue=t, yvalue=self.__rad_to_deg(true_state.omega[0,0]))
            self._plotter.add_data_point(plot_id='omega_y', data_label='omega_y', xvalue=t, yvalue=self.__rad_to_deg(true_state.omega[1,0]))
            self._plotter.add_data_point(plot_id='omega_z', data_label='omega_z', xvalue=t, yvalue=self.__rad_to_deg(true_state.omega[2,0]))

    def update(self, true_state, estimated_state, commanded_state, commanded_rates, ts):

        #convert quaternions to euler angles


        phi, theta, psi = quaternion_to_euler(estimated_state[6:10])
        estimated_st_euler = np.concatenate((estimated_state[0:6], np.array([[phi, theta, psi]]).T, estimated_state[10:13]))
        estimated_st_euler = np.squeeze(estimated_st_euler)

        phi, theta, psi = quaternion_to_euler(commanded_state[6:10])
        commanded_st_euler = np.concatenate((commanded_state[0:6], np.array([[phi, theta, psi]]).T))
        commanded_st_euler = np.squeeze(commanded_st_euler)

        self.plotter.add_vector_measurement('true_state', true_st_euler, self.time)
        self.plotter.add_vector_measurement('estimated_state', estimated_st_euler, self.time)
        self.plotter.add_vector_measurement('desired_state', commanded_st_euler, self.time)
        # self.plotter.add_vector_measurement('commanded_rates', commanded_rates, self.time)

        self.tick()

        # increment time
        self.time += ts

    def tick(self):
        # Update and display the plot
        self.plotter.update_plots()