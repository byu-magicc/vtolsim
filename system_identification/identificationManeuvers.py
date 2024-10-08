#this file creates the maneuver functions that are needed to creates the maneuvers for the vehicle

#imports the delta message class
from message_types.quad.msg_delta import MsgDelta

#imports the trim values
import models.quad.trimValues as TRIM

from models.quad.trimValues import trimDelta

#imports the message state class
from message_types.quad.msg_state import MsgState

#imports the signals class 
from tools.signals import Signals

#imports the signal types enumeration
from tools.signals import signalTypes
from enum import Enum

#imports the autopilot, which makes adjustments in between individual maneuvers
from controllers.quad.lqr_fixedWing import Autopilot

from message_types.quad.msg_autopilot_fixedWing import MsgAutopilotFixedWing
from message_types.quad.msg_state import MsgState

import parameters.quad.simulation_parameters as SIM

import numpy as np

import copy

#defines an enumeration corresponding to the 
class controlSurface(Enum):
    aileron = 0
    elevator = 1
    rudder = 2
    throttle = 3



#creates the identification maneuvers class
class individualManeuver:
    #creates the init function
    def __init__(self, 
                 Ts,
                 amplitude = 1.0,
                 start_frequency = 1.0,
                 start_time = 0.0,
                 signalDuration = 1.0,
                 totalDuration = 2.0,
                 dc_offset = 0.0,
                 phase = 0.0,
                 incremental_frequency = 0.0,
                 chosenControlSurface: controlSurface = controlSurface.aileron,
                 signalType: signalTypes = signalTypes.doublet):
        
        #stores the time sample rate
        self.Ts = Ts

        #instantiates the signal class
        self.signal = Signals(amplitude=amplitude,
                              start_frequency=start_frequency,
                              start_time=start_time,
                              duration=signalDuration,
                              dc_offset=dc_offset,
                              phase=phase,
                              incremental_frequency=incremental_frequency,
                              signalType=signalType)
        
        #creates the delta output object
        self.deltaOutput = MsgDelta()

        #gets the signal duration count
        self.signalDurationCount = int(signalDuration/self.Ts)

        #gets the total duration count
        self.totalDurationCount = int(totalDuration/self.Ts)

        #creates the counter variable, used for keeping time
        self.counter = 0

        #saves the control surface
        self.chosenControlSurface = chosenControlSurface

        #creates the maneuver completed flag
        self.maneuverCompleted = False

    #creates the update function
    def update(self):

        #first sets the delta output to trim
        deltaOutput = trimDelta


        #goes through and gets the trim for the chosen control surface
        if self.chosenControlSurface == controlSurface.aileron:
            trimOutput = TRIM.aileronTrim
        #case elevator
        elif self.chosenControlSurface == controlSurface.elevator:
            trimOutput = TRIM.elevatorTrim
        #case rudder
        elif self.chosenControlSurface == controlSurface.rudder:
            trimOutput = TRIM.rudderTrim
        #case throttle
        elif self.chosenControlSurface == controlSurface.throttle:
            trimOutput = TRIM.throttleTrim


        #automatically sets maneuver Completed to False. If it is completed, the first case will correct it.
        self.maneuverCompleted = False

        #case, the whole maneuver has been completed
        if self.counter >= self.totalDurationCount:
            #sets the flag to true
            self.maneuverCompleted = True
            #resets the counter
            self.counter = 0
            #sets the signal output to TRIM
            signalOutput = trimOutput
        #case the maneuver is still happening
        elif self.counter < self.totalDurationCount and self.counter < self.signalDurationCount:
            #sets the signal output to the maneuver update
            signalOutput = self.signal.update(time=(self.counter*self.Ts))
        #case the maneuver is done, and we are settling
        elif self.counter < self.totalDurationCount and self.counter >= self.signalDurationCount:
            #sets the signal output to trim, for settling action
            signalOutput = trimOutput


        #assigns the output for the correct control surface for the delta output
        #case aileron
        if self.chosenControlSurface == controlSurface.aileron:
            deltaOutput.aileron = signalOutput
        #case elevator
        elif self.chosenControlSurface == controlSurface.elevator:
            deltaOutput.elevator = signalOutput
        #case rudder
        elif self.chosenControlSurface == controlSurface.rudder:
            deltaOutput.rudder = signalOutput
        #case throttle
        elif self.chosenControlSurface == controlSurface.throttle:
            deltaOutput.throttle = signalOutput


        #increments the counter
        self.counter += 1
        
        #returns the output
        return deltaOutput
    

    #creates function to reset the counter
    def resetManeuver(self):
        self.counter = 0
        self.maneuverCompleted = False

    #creates function to get whether the maneuver has been completed
    def getManeuverCompleted(self):
        return self.maneuverCompleted


#creates the individual manevuers class, from which the 
#identification maneuvers class will be built over
class identificationManeuver:

    #creates the initialization function
    def __init__(self):
        #creates the array to store all of the individual maneuvers
        self.maneuverList = []
        #maneuver list length
        self.maneuverListLength = 0
        #creates the initialized flag
        self.initialized = False

        #creates variable for the index of the current individual maneuver being performed
        self.currentManeuver = 0

        #creates variable for all maneuvers completed
        self.allManeuversCompleted = False

        #instantiates the autopilot
        self.autopilotController = Autopilot(ts_control=SIM.ts_simulation)

        #creates the current state
        self.currentState = 0

        #creates flag for whether the autopilot portion has been completed
        #to stabilize each individual maneuver
        self.autopilotCompleted = False

        #sets the course error threshold (radians)
        self.courseErrorThreshold = np.radians(100)

        #sets the airspeed Error Threshold (meters / second)
        self.airspeedErrorThreshold = 100

        #sets the altitude error threshold (meters)
        self.altitudeErrorThreshold = 100

        #sets the time for the autopilot
        self.autopilotTime = 10.0

        #gets the autopilot count
        self.autopilotCount = int(self.autopilotTime/SIM.ts_simulation)

        #creates the autopilot counter
        self.autopilotCounter = 0

    
    #creates the update function, which is a state machine, to determine whether to use
    #the autopilot or to do the identification maneuver
    def update(self):

        #runs the update function for the respective controller
        deltaOutput = (self.maneuverList[self.currentManeuver]).update()

        #checks if the maneuver has been completed
        if (self.maneuverList[self.currentManeuver]).maneuverCompleted:
            #checks if we are currently at the last maneuver
            if self.currentManeuver == self.maneuverListLength - 1:
                #sets the all maneuvers completed flag to true
                self.allManeuversCompleted = True
            #otherwise we increment the current maneuver
            else:
                self.currentManeuver += 1


        #returns the delta Output
        return deltaOutput

    #creates function to add individual maneuvers to the full identification maneuver
    def addManeuver(self, maneuver: individualManeuver):
        #adds the maneuver
        self.maneuverList.append(maneuver)
        #increments the list length
        self.maneuverListLength += 1

    #creates function to get the maneuver
    def getManeuver(self, maneuverIndex: int):
        return self.maneuverList[maneuverIndex]

    #creates function to reset the whole identification maneuver
    def reset(self):
        self.allManeuversCompleted = False
        self.currentManeuver = 0

        #iterates through and resets all of the sub maneuvers
        for i in range(self.maneuverListLength):
            (self.maneuverList[i]).resetManeuver()

    #gets completion status
    def completionStatus(self):
        return self.allManeuversCompleted