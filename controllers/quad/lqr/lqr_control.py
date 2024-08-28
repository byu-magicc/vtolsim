#creates the two seperate lqr controllers. I am going to do two things as preliminaries to making the 
#whole simulation thing run. In this case, I am going to create two LQR controllers, one for VTOL and
#one for the standard plane. I first want to get the simulation running on each of them individually
#before I start building the rest of the controller

import numpy as np
from scipy.linalg import solve_continuous_are, inv
from tools.transfer_function import TransferFunction
import parameters.quad.fixed_wing_lqr_parameters as AP

#creates the autopilot for the standard plane configuration 
class LQR_Autopilot_Fixed_Wing:

    #creates the initialization function
    def __init__(self, ts_control):
        #saves the time sample
        self.Ts = ts_control
        self.yaw_damper = TransferFunction(
                        num=np.array([[AP.yaw_damper_kr, 0]]),
                        den=np.array([[1, AP.yaw_damper_p_wo]]),
                        Ts=ts_control)
        
        #creates the integrators for error for this system
        self.courseErrorIntegrator = 0.0
        self.altitudeErrorIntegrator = 0.0
        self.airspeedErrorIntegrator = 0.0
        self.errorCourseD1 = 0.0
        self.errorAltitudeD1 = 0.0
        self.errorAirspeedD1 = 0.0

                
