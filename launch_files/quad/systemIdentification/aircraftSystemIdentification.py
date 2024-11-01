#implements the function for system identification

#this file implements the parameter estimation algorithm, which has been refactored

import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))


#imports the numpy class
import numpy as np
#imports the matplotlib library
import matplotlib.pyplot as plt


#imports the parameters and libraries defined and written in our other files 
#imports the simulation parameters
import parameters.quad.simulation_parameters as SIM
from tools.signals import Signals
#gets the mav dynamics, which takes into account the sensors
from models.quad.quad_dynamics import QuadDynamics
import parameters.quad.anaconda_parameters as QUAD

#gets the view manager
from viewers.quad.view_manager import ViewManager
#imports the trim values
import models.quad.trimValues as TRIM
#imports the state message
from message_types.quad.msg_state import MsgState

from system_identification.parameterEstimatorController import parameterEstimatorController

from system_identification.identificationManeuvers import identificationManeuver
from system_identification.identificationManeuvers import individualManeuver
from system_identification.identificationManeuvers import controlSurface

from system_identification.aerodynamicsEstimator import AerodynamicsEstimator
from system_identification.aerodynamicsEstimator import ChosenAerodynamicSet

from message_types.quad.msg_aerodynamics import MsgAerodynamics
from message_types.quad.msg_aerodynamics import ChosenAerodynamicSet

from tools.signals import signalTypes

import pandas as pd

from models.quad.trimValues import trimDelta


#############################################################################################################
#constructs the first maneuver and estimator
firstManeuverMessage = MsgAerodynamics()

firstManeuverFirstChosenSubmessage = ChosenAerodynamicSet.C_Y
firstManeuverSecondChosenSubmessage = ChosenAerodynamicSet.C_l
firstManeuverThirdChosenSubmessage = ChosenAerodynamicSet.C_n

#appends each of the submessages
firstManeuverMessage.addAerodynamicMessage(aerodynamicSubmessageType=firstManeuverFirstChosenSubmessage)
firstManeuverMessage.addAerodynamicMessage(aerodynamicSubmessageType=firstManeuverSecondChosenSubmessage)
firstManeuverMessage.addAerodynamicMessage(aerodynamicSubmessageType=firstManeuverThirdChosenSubmessage)

#creates the first aerodynamics estimator
firstManeuverAerodynamicsEstimator = AerodynamicsEstimator(aerodynamicMessage=firstManeuverMessage)



firstManeuverFirstSubmaneuver = individualManeuver(Ts=SIM.ts_simulation,
                                                   amplitude=0.3,
                                                   signalDuration=0.2,
                                                   totalDuration=2.0,
                                                   dc_offset=TRIM.aileronTrim,
                                                   chosenControlSurface=controlSurface.aileron,
                                                   signalType=signalTypes.doublet)

#gets the second submaneuver
firstManeuverSecondSubmaneuver = individualManeuver(Ts=SIM.ts_simulation,
                                                   amplitude=0.3,
                                                   signalDuration=0.2,
                                                   totalDuration=2.0,
                                                   dc_offset=TRIM.rudderTrim,
                                                   chosenControlSurface=controlSurface.rudder,
                                                   signalType=signalTypes.doublet)

firstManeuverIdentificationManeuver = identificationManeuver()
#I want 5 sets fo the identification maneuver.
#because I'm feeling lazy, I'm going to just add them multiple times
firstManeuverIdentificationManeuver.addManeuver(maneuver=firstManeuverFirstSubmaneuver)
firstManeuverIdentificationManeuver.addManeuver(maneuver=firstManeuverSecondSubmaneuver)
firstManeuverIdentificationManeuver.addManeuver(maneuver=firstManeuverFirstSubmaneuver)
firstManeuverIdentificationManeuver.addManeuver(maneuver=firstManeuverSecondSubmaneuver)
firstManeuverIdentificationManeuver.addManeuver(maneuver=firstManeuverFirstSubmaneuver)
firstManeuverIdentificationManeuver.addManeuver(maneuver=firstManeuverSecondSubmaneuver)



#creates the parameter estimator controller
controller = parameterEstimatorController(Ts=SIM.ts_simulation)

controller.addEstimatorsManeuvers(estimator=firstManeuverAerodynamicsEstimator, maneuver=firstManeuverIdentificationManeuver)
#############################################################################################################

#############################################################################################################
#second maneuver and estimator section
secondManeuverMessage = MsgAerodynamics()

secondManeuverFirstSubmessage = ChosenAerodynamicSet.C_L_D
secondManeuverSecondSubmessage = ChosenAerodynamicSet.C_m

#adds each of the submessages
secondManeuverMessage.addAerodynamicMessage(aerodynamicSubmessageType=secondManeuverFirstSubmessage)
secondManeuverMessage.addAerodynamicMessage(aerodynamicSubmessageType=secondManeuverSecondSubmessage)

secondManeuverAerodynamicsEstimator = AerodynamicsEstimator(aerodynamicMessage=secondManeuverMessage)

#creates the submaneuvers
secondManeuverFirstSubmaneuver = individualManeuver(Ts=SIM.ts_simulation,
                                                    amplitude=0.3,
                                                    signalDuration=10.0,
                                                    totalDuration=15.0,
                                                    start_frequency=1.0,
                                                    incremental_frequency=2.0,
                                                    dc_offset=TRIM.elevatorTrim,
                                                    chosenControlSurface=controlSurface.elevator,
                                                    signalType=signalTypes.sinusoid)

secondManeuverIdentificationManeuver = identificationManeuver()

secondManeuverIdentificationManeuver.addManeuver(maneuver=secondManeuverFirstSubmaneuver)

#adds the second estimator and maneuvers to the list
controller.addEstimatorsManeuvers(estimator=secondManeuverAerodynamicsEstimator, maneuver=secondManeuverIdentificationManeuver)

#############################################################################################################




quad = QuadDynamics(SIM.ts_simulation)
#instantiates the viewers
viewers = ViewManager(data=True,
                      animation=True)

#gets the autopilot command
from message_types.quad.msg_autopilot_fixedWing import MsgAutopilotFixedWing
#instantiates the commands
commands = MsgAutopilotFixedWing()
Va_command = Signals(dc_offset=30.0,
                     amplitude=0.0,
                     start_time=0.0,
                     start_frequency = 0.01)
h_command = Signals(dc_offset=100.0,
                    amplitude=0.0,
                    start_time=0.0,
                    start_frequency=0.02)
chi_command = Signals(dc_offset=np.radians(0.0),
                      amplitude=np.radians(0.0),
                      start_time=0.0,
                      start_frequency=0.015)

#creates the commanded state variable
commandedState = MsgState()

#initializes the sim start and end times
sim_time = SIM.start_time
end_time = SIM.end_time

deltaArray = np.ndarray((8,0))

rotorForcesArray = np.ndarray((5,0))

while sim_time < end_time:

    #gets the autopilot commands
    commands.airspeed_command = Va_command.polynomial(sim_time)
    commands.course_command = chi_command.polynomial(sim_time)
    commands.altitude_command = h_command.polynomial(sim_time)

    (commandedState.pos)[2][0] = -commands.altitude_command


    commandedState.chi = commands.course_command
    commandedState.Va = commands.airspeed_command

    #gets the measurements
    measurements = quad.sensors()

    thrust = 1.0

    #gets the control output
    delta = controller.update(measurements, commands, quad.true_state, thrust)

    deltaArrayTemp = np.array([[delta.elevator],
                               [delta.aileron],
                               [delta.rudder],
                               [delta.forwardThrottle],
                               [delta.verticalThrottle_1],
                               [delta.verticalThrottle_2],
                               [delta.verticalThrottle_3],
                               [delta.verticalThrottle_4]])
    
    deltaArray = np.concatenate((deltaArray, deltaArrayTemp), axis=1)

    current_wind = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

    quad.update(delta, current_wind)

    rotorForces = quad.getCurrentThrusts()
    #concatenates onto the rotor forces array
    rotorForcesArray = np.concatenate((rotorForcesArray, rotorForces), axis=1)

    viewers.update(sim_time=sim_time,
                   true_state=quad.true_state,
                   estimated_state=quad.true_state,
                   commanded_state=quad.true_state,
                   delta=delta,
                   measurements=measurements)

    sim_time += SIM.ts_simulation


dataFrame1 = pd.DataFrame(data=deltaArray)
dataFrame1.to_csv("//home//dben1182//Documents//vtolsim//launch_files//quad//systemIdentification//sysIDDeltaOutput.csv", index=False, header=False)

dataFrame2 = pd.DataFrame(data=rotorForcesArray)
dataFrame2.to_csv("//home//dben1182//Documents//vtolsim//launch_files//quad//systemIdentification//sysIDRotorForcesOutput.csv", index=False, header=False)
