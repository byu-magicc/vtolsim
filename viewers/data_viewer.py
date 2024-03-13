"""
data_viewer
    - Update history:
        12/17/2018 - RWB
        1/14/2019 - RWB
        2/27/2020 - RWB
        1/19/2023 - David Christensen
        7/13/2023 - RWB
        11/20/2023 - RWB
"""
import numpy as np
from viewers.plotter import Plotter
from tools.wrap import wrap
from tools.rotations import rotation_to_euler

class DataViewer:
    def __init__(self, app,  dt = 0.01,
                 time_window_length = 30, # number of data points plotted at a time
                 plot_period = 0.2, # time interval between a plot update
                 data_recording_period = 0.1): # time interval between recording a data update
        self._dt = dt
        self._data_window_length= time_window_length/data_recording_period
        self._update_counter = 0
        self._plots_per_row = 5
        self._plotter = Plotter(app=app, plots_per_row=self._plots_per_row)  # plot last time_window seconds of data
        self._plot_period = plot_period
        self._data_recording_period = data_recording_period
        self._plot_delay = 0
        self._data_recording_delay = 0
        self._time = 0
        #define colors
        truth_color = (0,255,0)
        #truth_color = (160,202,111)
        #truth_color = (124,230,167)
        estimate_color = (255,0,0)
        #estimate_color = (255,150,150)
        #estimate_color = (255,154,111)
        #command_color = (0,0,255)
        command_color = (100,100,255)
        # define first row
        self._plotter.create_plot_widget(plot_id='north', xlabel='Time (s)', ylabel='north (m)',
                                        window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='east', xlabel='Time (s)', ylabel='east (m)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='altitude', xlabel='Time (s)', ylabel='altitude (m)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='throttle_right', xlabel='Time (s)', ylabel='throttle_right',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='throttle_left', xlabel='Time (s)', ylabel='throttle_left',
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
        self._plotter.create_data_set(plot_id="throttle_right", data_label="throttle_right", data_color=truth_color)
        self._plotter.create_data_set(plot_id="throttle_left", data_label="throttle_left", data_color=truth_color)

        # define second row
        self._plotter.create_plot_widget(plot_id='u', xlabel='Time (s)', ylabel='u (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='v', xlabel='Time (s)', ylabel='v (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='w', xlabel='Time (s)', ylabel='w (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='motor_right', xlabel='Time (s)', ylabel='motor_right (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='motor_left', xlabel='Time (s)', ylabel='motor_left (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="u", data_label="u", data_color=truth_color)
        self._plotter.create_data_set(plot_id="u", data_label="u_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="u", data_label="u_c", data_color=command_color)
        self._plotter.create_data_set(plot_id="v", data_label="v", data_color=truth_color)
        self._plotter.create_data_set(plot_id="v", data_label="v_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="v", data_label="v_c", data_color=command_color)
        self._plotter.create_data_set(plot_id="w", data_label="w", data_color=truth_color)
        self._plotter.create_data_set(plot_id="w", data_label="w_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="w", data_label="w_c", data_color=command_color)
        self._plotter.create_data_set(plot_id="motor_right", data_label="motor_right", data_color=truth_color)
        self._plotter.create_data_set(plot_id="motor_left", data_label="motor_left", data_color=truth_color)
        
        # define third row
        self._plotter.create_plot_widget(plot_id='roll', xlabel='Time (s)', ylabel='roll (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='pitch', xlabel='Time (s)', ylabel='pitch (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='yaw', xlabel='Time (s)', ylabel='yaw (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='throttle_rear', xlabel='Time (s)', ylabel='throttle_rear',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='rudder', xlabel='Time (s)', ylabel='rudder',
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
        self._plotter.create_data_set(plot_id="throttle_rear", data_label="throttle_rear", data_color=truth_color)            
        self._plotter.create_data_set(plot_id="rudder", data_label="rudder", data_color=truth_color)

        # define fourth row
        self._plotter.create_plot_widget(plot_id='omega_x', xlabel='Time (s)', ylabel='omega_x (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='omega_y', xlabel='Time (s)', ylabel='omega_y (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='omega_z', xlabel='Time (s)', ylabel='omega_z (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='elevator', xlabel='Time (s)', ylabel='elevator',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='aileron', xlabel='Time (s)', ylabel='aileron',
                                window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="omega_x", data_label="omega_x", data_color=truth_color)
        self._plotter.create_data_set(plot_id="omega_x", data_label="omega_x_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="omega_y", data_label="omega_y", data_color=truth_color)
        self._plotter.create_data_set(plot_id="omega_y", data_label="omega_y_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="omega_z", data_label="omega_z", data_color=truth_color)
        self._plotter.create_data_set(plot_id="omega_z", data_label="omega_z_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="elevator", data_label="elevator", data_color=truth_color)
        self._plotter.create_data_set(plot_id="aileron", data_label="aileron", data_color=truth_color)

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
            phi, theta, psi = rotation_to_euler(true_state.R)
            self._plotter.add_data_point(plot_id='north', data_label='north', xvalue=t, yvalue=true_state.pos[0,0])
            self._plotter.add_data_point(plot_id='east', data_label='east', xvalue=t, yvalue=true_state.pos[1,0])
            self._plotter.add_data_point(plot_id='altitude', data_label='altitude', xvalue=t, yvalue=-true_state.pos[2,0])
            self._plotter.add_data_point(plot_id='throttle_right', data_label='throttle_right', xvalue=t, yvalue=delta.throttle_right)
            self._plotter.add_data_point(plot_id='throttle_left', data_label='throttle_left', xvalue=t, yvalue=delta.throttle_left)
            self._plotter.add_data_point(plot_id='u', data_label='u', xvalue=t, yvalue=true_state.vel[0,0])
            self._plotter.add_data_point(plot_id='v', data_label='v', xvalue=t, yvalue=true_state.vel[1,0])
            self._plotter.add_data_point(plot_id='w', data_label='w', xvalue=t, yvalue=true_state.vel[2,0])
            self._plotter.add_data_point(plot_id='motor_right', data_label='motor_right', xvalue=t, yvalue=self.__rad_to_deg(true_state.motor_angle[0,0]))
            self._plotter.add_data_point(plot_id='motor_left', data_label='motor_left', xvalue=t, yvalue=self.__rad_to_deg(true_state.motor_angle[1,0]))
            self._plotter.add_data_point(plot_id='roll', data_label='roll', xvalue=t, yvalue=self.__rad_to_deg(phi))
            self._plotter.add_data_point(plot_id='pitch', data_label='pitch', xvalue=t, yvalue=self.__rad_to_deg(theta))
            self._plotter.add_data_point(plot_id='yaw', data_label='yaw', xvalue=t, yvalue=self.__rad_to_deg(psi))
            self._plotter.add_data_point(plot_id='throttle_rear', data_label='throttle_rear', xvalue=t, yvalue=delta.throttle_rear)
            self._plotter.add_data_point(plot_id='rudder', data_label='rudder', xvalue=t, yvalue=delta.rudder)
            self._plotter.add_data_point(plot_id='omega_x', data_label='omega_x', xvalue=t, yvalue=self.__rad_to_deg(true_state.omega[0,0]))
            self._plotter.add_data_point(plot_id='omega_y', data_label='omega_y', xvalue=t, yvalue=self.__rad_to_deg(true_state.omega[1,0]))
            self._plotter.add_data_point(plot_id='omega_z', data_label='omega_z', xvalue=t, yvalue=self.__rad_to_deg(true_state.omega[2,0]))
            self._plotter.add_data_point(plot_id='elevator', data_label='elevator', xvalue=t, yvalue=delta.elevator)
            self._plotter.add_data_point(plot_id='aileron', data_label='aileron', xvalue=t, yvalue=delta.aileron)
        #add the estimated state data
        if estimated_state != None:
            phi, theta, psi = rotation_to_euler(estimated_state.R)
            self._plotter.add_data_point(plot_id='north', data_label='north_e', xvalue=t, yvalue=estimated_state.pos[0,0])
            self._plotter.add_data_point(plot_id='east', data_label='east_e', xvalue=t, yvalue=estimated_state.pos[1,0])
            self._plotter.add_data_point(plot_id='altitude', data_label='altitude_e', xvalue=t, yvalue=-estimated_state.pos[2,0])
            self._plotter.add_data_point(plot_id='u', data_label='u_e', xvalue=t, yvalue=estimated_state.vel[0,0])
            self._plotter.add_data_point(plot_id='v', data_label='v_e', xvalue=t, yvalue=estimated_state.vel[1,0])
            self._plotter.add_data_point(plot_id='w', data_label='w_e', xvalue=t, yvalue=estimated_state.vel[2,0])
            self._plotter.add_data_point(plot_id='roll', data_label='roll_e', xvalue=t, yvalue=self.__rad_to_deg(phi))
            self._plotter.add_data_point(plot_id='pitch', data_label='pitch_e', xvalue=t, yvalue=self.__rad_to_deg(theta))
            self._plotter.add_data_point(plot_id='yaw', data_label='yaw_e', xvalue=t, yvalue=self.__rad_to_deg(psi))
            self._plotter.add_data_point(plot_id='omega_x', data_label='omega_x_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.omega[0,0]))
            self._plotter.add_data_point(plot_id='omega_y', data_label='omega_y_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.omega[1,0]))
            self._plotter.add_data_point(plot_id='omega_z', data_label='omega_z_e', xvalue=t, yvalue=self.__rad_to_deg(estimated_state.omega[2,0]))
        #add the commanded state data
        if commanded_state != None:
            phi, theta, psi = rotation_to_euler(commanded_state.R)
            self._plotter.add_data_point(plot_id='north', data_label='north_c', xvalue=t, yvalue=commanded_state.pos[0,0])
            self._plotter.add_data_point(plot_id='east', data_label='east_c', xvalue=t, yvalue=commanded_state.pos[1,0])
            self._plotter.add_data_point(plot_id='altitude', data_label='altitude_c', xvalue=t, yvalue=-commanded_state.pos[2,0])
            self._plotter.add_data_point(plot_id='u', data_label='u_c', xvalue=t, yvalue=commanded_state.vel[0,0])
            self._plotter.add_data_point(plot_id='v', data_label='v_c', xvalue=t, yvalue=commanded_state.vel[1,0])
            self._plotter.add_data_point(plot_id='w', data_label='w_c', xvalue=t, yvalue=commanded_state.vel[2,0])
            self._plotter.add_data_point(plot_id='roll', data_label='roll_c', xvalue=t, yvalue=self.__rad_to_deg(phi))
            self._plotter.add_data_point(plot_id='pitch', data_label='pitch_c', xvalue=t, yvalue=self.__rad_to_deg(theta))
            self._plotter.add_data_point(plot_id='yaw', data_label='yaw_c', xvalue=t, yvalue=self.__rad_to_deg(psi))

    def process_app(self):
        self._plotter.process_app(0)

    def __update_plot(self):
        self._plotter.update_plots()

    def close_data_viewer(self):
        self._plotter.close_window()

    def save_plot_image(self, plot_name):
        self._plotter.save_image(plot_name)

    def __rad_to_deg(self, radians):
        rad = wrap(radians,0)
        return rad*180/np.pi


