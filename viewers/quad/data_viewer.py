import numpy as np
from viewers.quad.plotter import Plotter
from tools.wrap import wrap
from tools.rotations import rotation_to_euler
from message_types.quad.msg_state import MsgState
from message_types.quad.msg_delta import MsgDelta


#creates the data viewer class
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
        
        #-------------------------------------First Row-------------------------------------------------
        # define first row
        self._plotter.create_plot_widget(plot_id='north', xlabel='Time (s)', ylabel='north (m)',
                                        window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='u', xlabel='Time (s)', ylabel='u (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='roll', xlabel='Time (s)', ylabel='roll (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='p', xlabel='Time (s)', ylabel='p (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='Va', xlabel='Time (s)', ylabel='Va (m/s)',
                                       window_length=self._data_window_length)
        #north data sets
        self._plotter.create_data_set(plot_id="north", data_label="north", data_color=truth_color)
        self._plotter.create_data_set(plot_id="north", data_label="north_e", data_color=estimate_color) 
        self._plotter.create_data_set(plot_id="north", data_label="north_c", data_color=command_color) 
        #u data sets
        self._plotter.create_data_set(plot_id="u", data_label="u", data_color=truth_color)
        self._plotter.create_data_set(plot_id="u", data_label="u_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="u", data_label="u_c", data_color=command_color)
        #phi data sets
        self._plotter.create_data_set(plot_id="roll", data_label="roll", data_color=truth_color)
        self._plotter.create_data_set(plot_id="roll", data_label="roll_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="roll", data_label="roll_c", data_color=command_color)
        #p data sets
        self._plotter.create_data_set(plot_id="p", data_label="p", data_color=truth_color)
        self._plotter.create_data_set(plot_id="p", data_label="p_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="p", data_label="p_c", data_color=command_color)
        #Va airspeed data sets
        self._plotter.create_data_set(plot_id="Va", data_label="Va", data_color=truth_color)
        self._plotter.create_data_set(plot_id="Va", data_label="Va_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="Va", data_label="Va_c", data_color=command_color)


        #----------------------------------Second Row--------------------------------------------------
        # define second row
        self._plotter.create_plot_widget(plot_id='east', xlabel='Time (s)', ylabel='east (m)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='v', xlabel='Time (s)', ylabel='v (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='pitch', xlabel='Time (s)', ylabel='pitch (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='q', xlabel='Time (s)', ylabel='q (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='alpha', xlabel='Time (s)', ylabel='alpha (deg)',
                                       window_length=self._data_window_length)
        #East portion
        self._plotter.create_data_set(plot_id="east", data_label="east", data_color=truth_color)
        self._plotter.create_data_set(plot_id="east", data_label="east_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="east", data_label="east_c", data_color=command_color)
        #v portion
        self._plotter.create_data_set(plot_id="v", data_label="v", data_color=truth_color)
        self._plotter.create_data_set(plot_id="v", data_label="v_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="v", data_label="v_c", data_color=command_color)
        #theta portion
        self._plotter.create_data_set(plot_id="pitch", data_label="pitch", data_color=truth_color)
        self._plotter.create_data_set(plot_id="pitch", data_label="pitch_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="pitch", data_label="pitch_c", data_color=command_color)
        #q portion
        self._plotter.create_data_set(plot_id="q", data_label="q", data_color=truth_color)
        self._plotter.create_data_set(plot_id="q", data_label="q_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="q", data_label="q_c", data_color=command_color)
        #alpha portion
        self._plotter.create_data_set(plot_id="alpha", data_label="alpha", data_color=truth_color)
        self._plotter.create_data_set(plot_id="alpha", data_label="alpha_e", data_color=estimate_color)        


        #----------------------------------------third row----------------------------------------------
        # define third row
        self._plotter.create_plot_widget(plot_id='altitude', xlabel='Time (s)', ylabel='altitude (m)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='w', xlabel='Time (s)', ylabel='w (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='yaw', xlabel='Time (s)', ylabel='yaw (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='r', xlabel='Time (s)', ylabel='r (deg/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='beta', xlabel='Time (s)', ylabel='beta (deg)',
                                       window_length=self._data_window_length)
        #altitude portion
        self._plotter.create_data_set(plot_id="altitude", data_label="altitude", data_color=truth_color)
        self._plotter.create_data_set(plot_id="altitude", data_label="altitude_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="altitude", data_label="altitude_c", data_color=command_color)
        #w portion
        self._plotter.create_data_set(plot_id="w", data_label="w", data_color=truth_color)
        self._plotter.create_data_set(plot_id="w", data_label="w_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="w", data_label="w_c", data_color=command_color)
        #yaw portion
        self._plotter.create_data_set(plot_id="yaw", data_label="yaw", data_color=truth_color)
        self._plotter.create_data_set(plot_id="yaw", data_label="yaw_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="yaw", data_label="yaw_c", data_color=command_color)
        #r portion
        self._plotter.create_data_set(plot_id="r", data_label="r", data_color=truth_color)
        self._plotter.create_data_set(plot_id="r", data_label="r_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="r", data_label="r_c", data_color=command_color)
        #beta portion
        self._plotter.create_data_set(plot_id="beta", data_label="beta", data_color=truth_color)
        self._plotter.create_data_set(plot_id="beta", data_label="beta_e", data_color=estimate_color)



        #-------------------------------------fourth row---------------------------------------------------
        # define fourth row
        self._plotter.create_plot_widget(plot_id='elevator', xlabel='Time (s)', ylabel='elevator',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='aileron', xlabel='Time (s)', ylabel='aileron',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='rudder', xlabel='Time (s)', ylabel='rudder',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='Vg', xlabel='Time (s)', ylabel='Vg (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='chi', xlabel='Time (s)', ylabel='chi (deg)',
                                window_length=self._data_window_length)
        #elevator portion
        self._plotter.create_data_set(plot_id="elevator", data_label="elevator", data_color=truth_color)
        #aileron portion
        self._plotter.create_data_set(plot_id="aileron", data_label="aileron", data_color=truth_color)
        #rudder portion
        self._plotter.create_data_set(plot_id="rudder", data_label="rudder", data_color=truth_color)
        #Vg portion
        self._plotter.create_data_set(plot_id="Vg", data_label="Vg", data_color=truth_color)
        self._plotter.create_data_set(plot_id="Vg", data_label="Vg_e", data_color=estimate_color)
        #Chi portion
        self._plotter.create_data_set(plot_id="chi", data_label="chi", data_color=truth_color)
        self._plotter.create_data_set(plot_id="chi", data_label="chi_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="chi", data_label="chi_c", data_color=command_color)


        #------------------------------------fifth row--------------------------------------------------------
        self._plotter.create_plot_widget(plot_id='delta_t_f', xlabel='Time (s)', ylabel='delta_t_f',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='delta_t_v1', xlabel='Time (s)', ylabel='delta_t_v1',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='delta_t_v2', xlabel='Time (s)', ylabel='delta_t_v2',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='delta_t_v3', xlabel='Time (s)', ylabel='delta_t_v3',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='delta_t_v4', xlabel='Time (s)', ylabel='delta_t_v4',
                                       window_length=self._data_window_length)       
        #delta_t_forward potion
        self._plotter.create_data_set(plot_id="delta_t_f", data_label="delta_t_f", data_color=truth_color)
        #delta_t_v1 portion
        self._plotter.create_data_set(plot_id="delta_t_v1", data_label="delta_t_v1", data_color=truth_color)
        #delta_t_v2 portion
        self._plotter.create_data_set(plot_id="delta_t_v2", data_label="delta_t_v2", data_color=truth_color)
        #delta_t_v3 portion
        self._plotter.create_data_set(plot_id="delta_t_v3", data_label="delta_t_v3", data_color=truth_color)
        #delta_t_v4 portion
        self._plotter.create_data_set(plot_id="delta_t_v4", data_label="delta_t_v4", data_color=truth_color)

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
        
    def __update_data(self, true_state: MsgState, estimated_state: MsgState, commanded_state: MsgState, delta: MsgDelta, currentTime):
        #add the true state data
        if true_state != None:
            phi, theta, psi = rotation_to_euler(true_state.R)
            #adds the actual positions
            self._plotter.add_data_point(plot_id='north', data_label='north', xvalue=currentTime, yvalue=true_state.pos[0,0])
            self._plotter.add_data_point(plot_id='east', data_label='east', xvalue=currentTime, yvalue=true_state.pos[1,0])
            self._plotter.add_data_point(plot_id='altitude', data_label='altitude', xvalue=currentTime, yvalue=-true_state.pos[2,0])
            #adds the body frame velocities
            self._plotter.add_data_point(plot_id='u', data_label='u', xvalue=currentTime, yvalue=true_state.vel[0,0])
            self._plotter.add_data_point(plot_id='v', data_label='v', xvalue=currentTime, yvalue=true_state.vel[1,0])
            self._plotter.add_data_point(plot_id='w', data_label='w', xvalue=currentTime, yvalue=true_state.vel[2,0])
            #adds the roll pitch and yaw
            self._plotter.add_data_point(plot_id='roll', data_label='roll', xvalue=currentTime, yvalue=self.__rad_to_deg(phi))
            self._plotter.add_data_point(plot_id='pitch', data_label='pitch', xvalue=currentTime, yvalue=self.__rad_to_deg(theta))
            self._plotter.add_data_point(plot_id='yaw', data_label='yaw', xvalue=currentTime, yvalue=self.__rad_to_deg(psi))
            #adds the p, q, and r
            self._plotter.add_data_point(plot_id='p', data_label='p', xvalue=currentTime, yvalue=self.__rad_to_deg(true_state.omega[0,0]))
            self._plotter.add_data_point(plot_id='q', data_label='q', xvalue=currentTime, yvalue=self.__rad_to_deg(true_state.omega[1,0]))
            self._plotter.add_data_point(plot_id='r', data_label='r', xvalue=currentTime, yvalue=self.__rad_to_deg(true_state.omega[2,0]))
            #adds the Airspeed, alpha, beta, Groundspeed, and course angle
            self._plotter.add_data_point(plot_id='Va', data_label='Va', xvalue=currentTime, yvalue=true_state.Va)
            self._plotter.add_data_point(plot_id='Vg', data_label='Vg', xvalue=currentTime, yvalue=true_state.Vg)
            self._plotter.add_data_point(plot_id='alpha', data_label='alpha', xvalue=currentTime, yvalue=true_state.alpha)
            self._plotter.add_data_point(plot_id='beta', data_label='beta', xvalue=currentTime, yvalue=true_state.beta)
            self._plotter.add_data_point(plot_id='chi', data_label='chi', xvalue=currentTime, yvalue=true_state.chi)


            self._plotter.add_data_point(plot_id='elevator', data_label='elevator', xvalue=currentTime, yvalue=delta.elevator)
            self._plotter.add_data_point(plot_id='aileron', data_label='aileron', xvalue=currentTime, yvalue=delta.aileron)
            self._plotter.add_data_point(plot_id='rudder', data_label='rudder', xvalue=currentTime, yvalue=delta.rudder)
            self._plotter.add_data_point(plot_id='delta_t_f', data_label='delta_t_f', xvalue=currentTime, yvalue=delta.forwardThrottle)
            self._plotter.add_data_point(plot_id='delta_t_v1', data_label='delta_t_v1', xvalue=currentTime, yvalue=delta.verticalThrottle_1)
            self._plotter.add_data_point(plot_id='delta_t_v2', data_label='delta_t_v2', xvalue=currentTime, yvalue=delta.verticalThrottle_2)
            self._plotter.add_data_point(plot_id='delta_t_v3', data_label='delta_t_v3', xvalue=currentTime, yvalue=delta.verticalThrottle_3)
            self._plotter.add_data_point(plot_id='delta_t_v4', data_label='delta_t_v4', xvalue=currentTime, yvalue=delta.verticalThrottle_4)


        #add the estimated state data
        if estimated_state != None:
            phi, theta, psi = rotation_to_euler(estimated_state.R)
            #gets the inertial frame positions
            self._plotter.add_data_point(plot_id='north', data_label='north_e', xvalue=currentTime, yvalue=estimated_state.pos[0,0])
            self._plotter.add_data_point(plot_id='east', data_label='east_e', xvalue=currentTime, yvalue=estimated_state.pos[1,0])
            self._plotter.add_data_point(plot_id='altitude', data_label='altitude_e', xvalue=currentTime, yvalue=-estimated_state.pos[2,0])
            #gets the body frame velocities
            self._plotter.add_data_point(plot_id='u', data_label='u_e', xvalue=currentTime, yvalue=estimated_state.vel[0,0])
            self._plotter.add_data_point(plot_id='v', data_label='v_e', xvalue=currentTime, yvalue=estimated_state.vel[1,0])
            self._plotter.add_data_point(plot_id='w', data_label='w_e', xvalue=currentTime, yvalue=estimated_state.vel[2,0])
            #gets the roll, pitch, and yaws
            self._plotter.add_data_point(plot_id='roll', data_label='roll_e', xvalue=currentTime, yvalue=self.__rad_to_deg(phi))
            self._plotter.add_data_point(plot_id='pitch', data_label='pitch_e', xvalue=currentTime, yvalue=self.__rad_to_deg(theta))
            self._plotter.add_data_point(plot_id='yaw', data_label='yaw_e', xvalue=currentTime, yvalue=self.__rad_to_deg(psi))
            #gets the p, q, and r
            self._plotter.add_data_point(plot_id='p', data_label='p_e', xvalue=currentTime, yvalue=self.__rad_to_deg(estimated_state.omega[0,0]))
            self._plotter.add_data_point(plot_id='q', data_label='q_e', xvalue=currentTime, yvalue=self.__rad_to_deg(estimated_state.omega[1,0]))
            self._plotter.add_data_point(plot_id='r', data_label='r_e', xvalue=currentTime, yvalue=self.__rad_to_deg(estimated_state.omega[2,0]))
            #gets the Airspeed, alpha, beta, groundspeed, and course angle
            self._plotter.add_data_point(plot_id='Va', data_label='Va_e', xvalue=currentTime, yvalue=estimated_state.Va)
            self._plotter.add_data_point(plot_id='Vg', data_label='Vg_e', xvalue=currentTime, yvalue=estimated_state.Vg)
            self._plotter.add_data_point(plot_id='alpha', data_label='alpha_e', xvalue=currentTime, yvalue=estimated_state.alpha)
            self._plotter.add_data_point(plot_id='beta', data_label='beta_e', xvalue=currentTime, yvalue=estimated_state.beta)
            self._plotter.add_data_point(plot_id='chi', data_label='chi_e', xvalue=currentTime, yvalue=estimated_state.chi)

        #add the commanded state data
        if commanded_state != None:
            phi, theta, psi = rotation_to_euler(commanded_state.R)
            #gets the inertial frame positions
            self._plotter.add_data_point(plot_id='north', data_label='north_c', xvalue=currentTime, yvalue=commanded_state.pos[0,0])
            self._plotter.add_data_point(plot_id='east', data_label='east_c', xvalue=currentTime, yvalue=commanded_state.pos[1,0])
            self._plotter.add_data_point(plot_id='altitude', data_label='altitude_c', xvalue=currentTime, yvalue=-commanded_state.pos[2,0])
            #gets the body frame velocities
            self._plotter.add_data_point(plot_id='u', data_label='u_c', xvalue=currentTime, yvalue=commanded_state.vel[0,0])
            self._plotter.add_data_point(plot_id='v', data_label='v_c', xvalue=currentTime, yvalue=commanded_state.vel[1,0])
            self._plotter.add_data_point(plot_id='w', data_label='w_c', xvalue=currentTime, yvalue=commanded_state.vel[2,0])
            #gets the roll, pitch, and yaws
            self._plotter.add_data_point(plot_id='roll', data_label='roll_c', xvalue=currentTime, yvalue=self.__rad_to_deg(phi))
            self._plotter.add_data_point(plot_id='pitch', data_label='pitch_c', xvalue=currentTime, yvalue=self.__rad_to_deg(theta))
            self._plotter.add_data_point(plot_id='yaw', data_label='yaw_c', xvalue=currentTime, yvalue=self.__rad_to_deg(psi))
            #gets the p, q, and r
            self._plotter.add_data_point(plot_id='p', data_label='p_c', xvalue=currentTime, yvalue=self.__rad_to_deg(commanded_state.omega[0,0]))
            self._plotter.add_data_point(plot_id='q', data_label='q_c', xvalue=currentTime, yvalue=self.__rad_to_deg(commanded_state.omega[1,0]))
            self._plotter.add_data_point(plot_id='r', data_label='r_c', xvalue=currentTime, yvalue=self.__rad_to_deg(commanded_state.omega[2,0]))
            #gets the Airspeed, alpha, beta, groundspeed, and course angle
            self._plotter.add_data_point(plot_id='Va', data_label='Va_c', xvalue=currentTime, yvalue=commanded_state.Va)
            self._plotter.add_data_point(plot_id='chi', data_label='chi_c', xvalue=currentTime, yvalue=commanded_state.chi)


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


