"""
vtolsim: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/1/2019 - Randy Beard
        4/15/2019 - BGM
        5/3/2019 - Randy Beard
"""

import numpy as np
import pyqtgraph.opengl as gl


class DrawTrajectory:
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
