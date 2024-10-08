
#imports the main autopilot
from controllers.quad.autopilot_fixedWing import Autopilot


#imports the identification maneuvers class
from system_identification.identificationManeuvers import identificationManeuver

#imports the aerodynamic estimator class
from system_identification.aerodynamicsEstimator import AerodynamicsEstimator

#imports the trim
from models.quad.trimValues import trimDelta

import parameters.quad.simulation_parameters as SIM


from message_types.quad.msg_sensors import MsgSensors
from message_types.quad.msg_state import MsgState
from message_types.quad.msg_delta import MsgDelta
from message_types.quad.msg_autopilot_fixedWing import MsgAutopilotFixedWing

import numpy as np


#the parameter estimator controller is the main overarching controller, which implements multiple maneuvers and 
#multiple estimators on top of one another, giving the delta commands, and then running the multiple aerodynamics estimators
class parameterEstimatorController:

    #creates the initialization function
    def __init__(self, Ts):
        
        self.Ts = Ts

        #instantiates the autopilot
        self.Autopilot = Autopilot(ts_control=Ts)

        #creates flag for whether the aerodynamicsEstimator list and the identificationManeuvers
        #list has had anything added to it
        self.estimatorAndManeuverInitialized = False

        #creates the Estimator list and the maneuver list
        self.aerodynamicsEstimatorList = [0]
        self.identificationManeuversList = [0]

        #variable stores which estimator/Maneuver we are using now.
        self.currentEstimatorManeuver = 0

        #stores the current state of the state machine
        self.currentState = 0

        #stores the number of estimators and maneuvers
        self.numEstimatorsManeuvers = 0

        #creates the delta, which is to be output
        self.delta = trimDelta

        #sets the course error (radians)
        self.courseErrorThreshold = np.radians(2)

        #sets the airspeed error (meters/second)
        self.airspeedErrorThreshold = 1.0

        #sets the altitude error threshold (meters)
        self.altitudeErrorThreshold = 1.0

        #sets the autopilot settling time
        self.autoSettleTime = 15.0

        #sets the autopilot settling count
        self.autoSettleCount = int(self.autoSettleTime/SIM.ts_simulation)

        #creates the counter for the autopilot settling
        self.autoSettleCounter = 0

        self.testCounter = 0


    
    #creates the update function, which is a state machine that 
    def update(self, sensorMeasurements: MsgSensors, command: MsgAutopilotFixedWing, state: MsgState, thrust):
        

        #case current state is initialization state
        if self.currentState == 0:
            #calls the reset maneuvers function
            self.resetIdentificationManeuvers()
            #increments the current state
            self.incrementCurrentState()
            #sets the output delta to trim
            self.delta = trimDelta

            #reset the auto settle counter
            self.autoSettleCounter = 0

        #case the current state is the autopilot reset state
        elif self.currentState == 1:

            #gets the altitude, the negative of the 3rd position slot
            altitude = -(state.pos)[2][0]

            #checks if we are outside of the course error threshold and the counter has settled
            #if so, we transition to the next state
            if np.abs(state.chi - command.course_command) <= self.courseErrorThreshold and\
               np.abs(altitude - command.altitude_command) <= self.altitudeErrorThreshold and\
               np.abs(state.Va - command.airspeed_command) <= self.airspeedErrorThreshold and\
               self.autoSettleCounter >= self.autoSettleCount:
                
                #increments the current state
                self.currentState += 1
                #sets the delta output to trim
                self.delta=trimDelta
                #resets the auto settle counter
                self.autoSettleCounter = 0
            #otherwise, we use the autopilot
            else:
                self.delta, commandedState = self.Autopilot.update(cmd=command, state=state)
                #increments the auto Settle Counter
                self.autoSettleCounter += 1
        #case current state is maneuvers state
        elif self.currentState == 2:

            #gets the delta output from the current maneuver
            self.delta = (self.identificationManeuversList[self.currentEstimatorManeuver]).update()

            #calls the update function for each aerodynamics estimator
            (self.aerodynamicsEstimatorList[self.currentEstimatorManeuver]).update(sensorMeasurements, state, self.delta, thrust)

            #gets whether the current maneuver has been completed
            if (self.identificationManeuversList[self.currentEstimatorManeuver]).completionStatus():
                #case we are at the end of the list
                if self.currentEstimatorManeuver == (self.numEstimatorsManeuvers - 1):
                    #goes to the reset state
                    self.currentState = 0
                    #prints the estimated Coefficients
                    firstCoefMessageArray = self.getCoefficients(0)
                    
                    #gets the Y array
                    yArray = (firstCoefMessageArray[0]).getCoefficients()
                    print("Y Coefficients: ")
                    print(yArray)

                    lArray = (firstCoefMessageArray[1]).getCoefficients()
                    print("l Coefficients: ")
                    print(lArray)

                    nArray = (firstCoefMessageArray[2]).getCoefficients()
                    print("n Coefficients: ")
                    print(nArray)

                    secondCoefMessageArray = self.getCoefficients(1)

                    LDArray =(secondCoefMessageArray[0]).getCoefficients()
                    print("L D Coefficients")
                    print(LDArray)

                    mArray = (secondCoefMessageArray[1]).getCoefficients()
                    print("m Coefficients")
                    print(mArray)

                #otherwise we increment the current estimator maneuver
                else:
                    self.currentEstimatorManeuver += 1

        if (self.testCounter*self.Ts >= 14.0) and (self.testCounter % 10 == 0):
            a=0

        self.testCounter += 1

        #returns the delta
        return self.delta



    #creates function to add identification maneuvers classes and Aerodynamics Estimator classes to the list for the parameter Estimator Controller
    def addEstimatorsManeuvers(self, estimator: AerodynamicsEstimator, maneuver: identificationManeuver):
        #case initialized
        if self.estimatorAndManeuverInitialized:
            #adds the estimator to the estimator list
            self.aerodynamicsEstimatorList.append(estimator)
            #adds the maneuver to the maneuver list
            self.identificationManeuversList.append(maneuver)
        #case uninitialized
        else:
            self.aerodynamicsEstimatorList[0] = estimator
            self.identificationManeuversList[0] = maneuver
            #sets the initialized flag to true
            self.estimatorAndManeuverInitialized = True

        #increments the number of estimators/maneuvers
        self.numEstimatorsManeuvers += 1
    
    #function to get the coefficients being estimated
    def getCoefficients(self, index):
        coefficientsArray = (self.aerodynamicsEstimatorList[index]).getCoefficients()
        return coefficientsArray


    #function which resets all of the identification maneuvers
    def resetIdentificationManeuvers(self):
        #iterates through all the maneuvers
        for i in range(self.numEstimatorsManeuvers):
            #calls the reset counter for each of them
            self.identificationManeuversList[i].reset()

    #function which increments the current state
    def incrementCurrentState(self):
        self.currentState += 1

    
    #function which increments the current estimator maneuver
    def incrementCurrentEstimatorManeuver(self):
        self.currentEstimatorManeuver += 1

    