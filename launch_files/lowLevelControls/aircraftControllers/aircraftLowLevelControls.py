#this file launches and runs the low level controller for just the airplane controls
#as in the elevator, aileron, rudder and forward throttle contorls

import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))
import numpy as np
import parameters.simulation_parameters as SIM
from models.quad_dynamics_control import QuadDynamicsControl
from controllers.autopilot_fixedWing import Autopilot

from viewers.view_manager import ViewManager
from message_types.msg_autopilot_fixedWing import MsgAutopilot
from tools.signals import Signals

from controllers.low_level_control import LowLevelControl_aircraftControl

#gets the wrench calculation
from controllers.forces_torques_derivatives import wrenchCalculation

import pandas as pd

quadrotorsExist = True

#instantiates the quad
quad = QuadDynamicsControl(Ts=SIM.ts_simulation, quadrotorsExist=quadrotorsExist)

#creates the view manager

#creates the view manager
viewers = ViewManager(animation=True, data=True)

wrenchCalculator = wrenchCalculation(quadrotorsExist=quadrotorsExist)

#creates the low level controller
lowLevelController = LowLevelControl_aircraftControl(ts=SIM.ts_simulation, torqueControl=True)

#reads in the data matrix from the file
wrenchDataObject = pd.read_csv("/home/benjamin/Documents/vtolsim/outputFiles/lowLevelController/fixedWingTest/wrenchReference.csv")

#gets the array
wrenchData = wrenchDataObject.to_numpy()

#gets the length of the wrench data
wrenchDataLength = (np.shape(wrenchData))[1]


#creates the counter to see where we are
counter = 0

while counter < wrenchDataLength:

    #gets the sim time
    sim_time = counter*SIM.ts_simulation

    #gets the partition of the wrenchData
    currentWrench = wrenchData[:,counter]

    wind = np.array([[0],[0],[0]])

    #creates the force desired
    Force_desired = np.array([[currentWrench.item(0)],
                              [currentWrench.item(1)]])

    tau_desired = np.array([[currentWrench.item(2)],
                            [currentWrench.item(3)],
                            [currentWrench.item(4)]])


    #################################################################################################################
    #most important part here
    #calls the wrench following controller
    delta = lowLevelController.update(f_d=Force_desired,
                                      state = quad.true_state,
                                      wind=wind,
                                      tau_desired=tau_desired)
    #################################################################################################################

    # -------physical system-------------
    current_wind = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]) # get the new wind vector
    quad.update(delta, current_wind)  # propagate the MAV dynamics   

    # ------- update viewers -------
    viewers.update(
        sim_time,
        true_state=quad.true_state,  # true states
        commanded_state=quad.true_state,  # commanded states
        estimated_state=quad.true_state,
        measurements=quad.true_state,
        delta=delta, # inputs to MAV
    )

    #increments the counter
    counter += 1



penpineappleapplepen = 0