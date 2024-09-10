#this file implements a force and torque following algorithm for the airplane in 
#fixed wing mode

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[4]))

import pandas as pd

import numpy as np
import parameters.quad.simulation_parameters as SIM

#imports the quad dynamics to create the plane model
from models.quad.quad_dynamics import QuadDynamics
#imports the low level controller for the Quad,
#which minimizes the mean squared error
from tools.signals import Signals

from controllers.quad.low_level_control import LowLevelControl_SurfacesShortened
from message_types.quad.msg_delta import MsgDelta
from viewers.quad.view_manager import ViewManager
from message_types.quad.msg_state import MsgState
import pandas as pd

from models.quad.trimValues import trimDelta
import models.quad.trimValues as TRIM


#reads in the csv file
wrenchInputClass = pd.read_csv('/home/dben1182/Documents/vtolsim/launch_files/quad/controllerTests/fixedWing/desiredWrench.csv',header=None, index_col=None)
wrenchInputVector = wrenchInputClass.values

#creates the controller
controller = LowLevelControl_SurfacesShortened(ts=SIM.ts_simulation)
#gets the quad
quad = QuadDynamics(ts=SIM.ts_simulation)
#creates the viewer
viewer = ViewManager(animation=True, data=True)

#creates the wind
wind = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T

actualForceTorque = np.ndarray((6,0))

deltaOutput = np.ndarray((3,0))

sim_time = SIM.start_time
end_time = SIM.end_time

counter = 0

while sim_time < end_time:

    #gets the individual vector
    wrenchInput = np.reshape(wrenchInputVector[:,counter], (6,1))
    

    forceInput = np.array([[wrenchInput[0][0]],
                           [wrenchInput[2][0]]])
    torqueInput = np.array([[wrenchInput[3][0]],
                            [wrenchInput[4][0]]])

    #calls the update function for the controller
    delta = controller.update(f_d=forceInput,
                              state=quad.true_state,
                              quad=quad,
                              tau_input=True,
                              tau_d=torqueInput)

    
    #updates the quadplane
    forceTorque = quad.update(delta=delta, wind=wind)

    #concatenates the actual measured force and torque output
    actualForceTorque = np.concatenate((actualForceTorque, forceTorque), axis=1)

    deltaOutput = np.concatenate((deltaOutput, np.array([[delta.elevator],[delta.aileron],[delta.forwardThrottle]])), axis=1)

    #updates the viewer
    viewer.update(sim_time=sim_time,
                  true_state=quad.true_state,
                  estimated_state=quad.true_state,
                  commanded_state=quad.true_state,
                  delta=delta,
                  measurements=None)



    counter += 1
    sim_time += SIM.ts_simulation


#writes out the actual force and torques
dataFrame1 = pd.DataFrame(actualForceTorque)
dataFrame1.to_csv("/home/dben1182/Documents/vtolsim/launch_files/quad/controllerTests/fixedWing/actualForceTorque.csv", index=False, header=False)


#writes out the deltas
dataFrame2 = pd.DataFrame(deltaOutput)
dataFrame2.to_csv("/home/dben1182/Documents/vtolsim/launch_files/quad/controllerTests/fixedWing/deltasActual.csv", index=False, header=False)