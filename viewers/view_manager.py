import pyqtgraph as pg
from viewers.quad_viewer import QuadViewer
from viewers.data_viewer import DataViewer

import numpy as np

#imports the simulation parameters
import parameters.simulation_parameters as SIM
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from message_types.msg_delta import MsgDelta

from PyQt5 import QtWidgets
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector


#creates the view manager class
class ViewManager:
    def __init__(self,
                 video: bool=False,
                 data: bool=False,
                 sensors: bool=False,
                 animation: bool=False,
                 save_plots: bool=False,
                 video_name: str=[]):
        self.video_flag = video
        self.data_plot_flag = data
        self.sensor_plot_flag = sensors
        self.animation_flag = animation
        self.save_plots_flag = save_plots
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('VTOL Viewer')
        self.window.setGeometry(0, 0, 1000, 1000)  # args: upper_left_x, upper_right_y, width, height
        # initialize video 
        # initialize the other visualization
        if self.animation_flag or self.data_plot_flag or self.sensor_plot_flag: 
            self.app = pg.QtWidgets.QApplication([]) 
            if self.animation_flag:
                self.vtol_view = QuadViewer(app=self.app, 
                                            dt=SIM.ts_simulation,
                                            plot_period=SIM.ts_plot_refresh)
            if self.data_plot_flag: 
                self.data_view = DataViewer(
                    app=self.app,
                    dt=SIM.ts_simulation,
                    plot_period=SIM.ts_plot_refresh, 
                    data_recording_period=SIM.ts_plot_record_data, 
                    time_window_length=30)

            if self.sensor_plot_flag: 
                self.sensor_view = SensorViewer(
                    app=self.app,
                    dt=SIM.ts_simulation, 
                    plot_period=SIM.ts_plot_refresh, 
                    data_recording_period=SIM.ts_plot_record_data, 
                    time_window_length=30)
                
    def update(self,
               sim_time: float,
               true_state: MsgState, 
               estimated_state: MsgState, 
               commanded_state: MsgState, 
               delta: MsgDelta,
               measurements: MsgSensors):
        if self.animation_flag: 
            self.vtol_view.update(true_state) 
        if self.data_plot_flag:
            self.data_view.update(
                true_state,  # true states
                estimated_state,  # estimated states
                commanded_state,  # commanded states
                delta)  # inputs to aircraft
        if self.sensor_plot_flag: 
            self.sensor_view.update(measurements)
        if self.animation_flag or self.data_plot_flag or self.sensor_plot_flag: 
            self.app.processEvents()
        if self.video_flag is True: 
            self.video.update(sim_time)
    
    def close(self, dataplot_name: str=[], sensorplot_name: str=[]):
        # Save an Image of the Plot
        if self.save_plots_flag:
            if self.data_plots_flag: 
                self.data_view.save_plot_image(dataplot_name)
            if self.sensor_plots_flag: 
                self.sensor_view.save_plot_image(sensorplot_name)
        if self.video_flag: 
            self.video.close()

    def addTrajectory(self, points):
        blue = np.array([[30, 144, 255, 255]])/255.
        self.trajectory = drawTrajectory(points, blue, self.window)




class drawTrajectory:
    def __init__(self, points, color, window):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = R @ np.copy(points)
        
        points = points.T
        self.color = color
        path_color = np.tile(color, (points.shape[0], 1))
        self.path_plot_object =  gl.GLLinePlotItem(pos=points,
                                                   color=path_color,
                                                   width=2,
                                                   antialias=True,
                                                   mode='line_strip')
        window.addItem(self.path_plot_object)

    def update(self, points):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = R @ np.copy(points)
        self.path_plot_object.setData(pos=points)
