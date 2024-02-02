"""
vtol_viewer: simple vtol viewer
        4/1/2019 - R.W. Beard
        4/15/2019 - BGM
        5/3/2019 - R.W. Beard
        11/16/2023 - R.W. Beard
"""
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector
from viewers.draw_vtol import DrawVtol

class VtolViewer():
    def __init__(self, app, dt = 0.01,
                 plot_period = 0.2): # time interval between a plot update
        # initialize Qt gui application and window
        self._dt = dt
        self._time = 0
        self._plot_period = plot_period
        self._plot_delay = 0
        self.app = app  # initialize QT, external so that only one QT process is running
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('VTOL Viewer')
        self.window.setGeometry(0, 0, 500, 500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(20, 20, 20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=20) # distance from center of plot to camera
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_() # bring window to the front
        self.plot_initialized = False # has the mav been plotted yet?
        self.vtol_plot = []

    def update(self, state):
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.vtol_plot = DrawVtol(state, self.window)
            # update the center of the camera view to the quadrotor location
            # defined in ENU coordinates
            view_location = Vector(state.pos.item(1), state.pos.item(0), -state.pos.item(2))
            self.window.opts['center'] = view_location
            # redraw
            self.app.processEvents()
            self.plot_initialized = True
        # else update drawing on all other calls to update()
        else:
            if self._plot_delay >= self._plot_period:
                self.vtol_plot.update(state)
                self._plot_delay = 0
                # update the center of the camera view to the vtol location
                # defined in ENU coordinates
                view_location = Vector(state.pos.item(1), state.pos.item(0), -state.pos.item(2))
                self.window.opts['center'] = view_location
                # redraw
                self.app.processEvents()
            self._plot_delay += self._dt

    def close(self):
        self.window.close()
