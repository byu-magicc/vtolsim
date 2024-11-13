#this file creates functions to record and measure performance of the airplane through the whole trajectory.
import sys
sys.path.append('..')

import numpy as np
import parameters.simulation_parameters as SIM

import pandas as pd

from message_types.msg_delta import MsgDelta

from tools.rotations import quaternion_to_euler

#our performance measures are: 
#1. Positional Error Integral
#2. Total Energy consumption (Integral of Power Consumption)



#the performance Measures super class has several subclasses, each of which perform the individual metric
#calculation, and then puts everything together in the bigger performanceMeasures super class
class performanceMeasures:

    #creates the init function
    def __init__(self, Ts):

        #creates an instance of the error tracking class
        self.posErrorTracker = positionErrorTracker(Ts=Ts)

        #instantiates the energy tracker
        self.energyTracker = energyTracker(Ts=Ts)

        self.deltaTracker = deltaTracker()

        self.stateTracker = stateTracker()


#class that tracks the position, the error, various norms, and the 
#total integral of the distance error
class positionErrorTracker:

    def __init__(self, Ts: float):

        self.Ts = Ts

        #creates array to store the positional vectors
        self.desiredPosition = np.ndarray((3,0))
        self.actualPosition = np.ndarray((3,0))
        #creates array to store the error position
        self.errorPosition = np.ndarray((3,0))

        #creates array to store the 1 and 2 norms of the positional error
        self.errorPosition1Norm = np.ndarray((1,0))
        self.errorPosition2Norm = np.ndarray((1,0))

        #sets the error integrals for 1 and 2 norms
        self.errorPosition1Norm_Integral = 0.0
        self.errorPosition2Norm_Integral = 0.0


    #creates the positional error tracking function
    def update(self, desiredPosition: np.ndarray, actualPosition: np.ndarray):
        
        #section to perform the calculations
        #reshapes the input vectors
        desiredPosition = np.reshape(desiredPosition, (3,1))
        actualPosition = np.reshape(actualPosition, (3,1))
        #gets the error position
        errorPosition = actualPosition - desiredPosition

        #calculates the 1 and 2 norms for the error position
        error_1Norm = np.linalg.norm(x=errorPosition, ord=1)
        error_2Norm = np.linalg.norm(x=errorPosition, ord=2)
        
        #section to store everything

        #updates the positional error tracker
        self.desiredPosition = np.concatenate((self.desiredPosition, desiredPosition), axis=1)
        self.actualPosition = np.concatenate((self.actualPosition, actualPosition), axis=1)
        #stores the error position
        self.errorPosition = np.concatenate((self.errorPosition, errorPosition), axis=1)

        #saves the normalizations
        self.errorPosition1Norm = np.concatenate((self.errorPosition1Norm, np.reshape(error_1Norm, (1,1))), axis=1)
        self.errorPosition2Norm = np.concatenate((self.errorPosition2Norm, np.reshape(error_2Norm, (1,1))), axis=1)


        #updates the integrals
        self.errorPosition1Norm_Integral += np.abs(error_1Norm*self.Ts)
        self.errorPosition2Norm_Integral += np.abs(error_2Norm*self.Ts)


    #gets the position norms
    def getErrorPositionNorms(self, degree: int = 2):
        #case degree is 1
        if degree == 1:
            return self.errorPosition1Norm
        elif degree == 2:
            return self.errorPosition2Norm
        else:
            return self.errorPosition2Norm

    #gets the integral norms
    def getErrorIntegralNorms(self, degree: int=2):
        if degree == 1:
            return self.errorPosition1Norm_Integral
        else:
            return self.errorPosition2Norm_Integral

    #defines functions to get the various 

    #creates functions to reset the trackers
    def resetTracker(self):
        #creates array to store the positional vectors
        self.desiredPosition = np.ndarray
        self.actualPosition = np.ndarray
        #creates array to store the error position
        self.errorPosition = np.ndarray

        #creates array to store the 1 and 2 norms of the positional error
        self.errorPosition1Norm = np.ndarray
        self.errorPosition2Norm = np.ndarray

        #sets the error integrals for 1 and 2 norms
        self.errorPosition1Norm_Integral = 0.0
        self.errorPosition2Norm_Integral = 0.0


#class that tracks the instantaneous power to the motors and records it
#and takes the integral to get total energy expenditure.
class energyTracker:

    def __init__(self, Ts: float):
        self.Ts = Ts

        #creates a vector to store the V_in, I_in, and P_in vectors
        self.Voltage_Array = np.ndarray((5,0))
        self.Current_Array = np.ndarray((5,0))
        self.Power_Array = np.ndarray((5,0))

        #gets the norm of the power 
        self.normPower_Array = np.ndarray((1,0))

        self.totalEnergyOut = 0.0

    #creates the update function
    def update(self, V_in: np.ndarray, I_in: np.ndarray, P_in: np.ndarray):

        V_in = np.reshape(V_in, (5,1))
        I_in = np.reshape(I_in, (5,1))
        P_in = np.reshape(P_in, (5,1))

        #concatenates them all in
        self.Voltage_Array = np.concatenate((self.Voltage_Array, V_in), axis=1)
        self.Current_Array = np.concatenate((self.Current_Array, I_in), axis=1)
        self.Power_Array = np.concatenate((self.Power_Array, P_in), axis=1)

        powerNorm = np.linalg.norm(P_in, ord=2)

        self.normPower_Array = np.concatenate((self.normPower_Array, np.array([[powerNorm]])), axis=1)

        #adds the new bit of energy
        #powerNorm (Joules/second) * Ts (seconds) = incrementalEnergy (Joules)
        self.totalEnergyOut += powerNorm*self.Ts

    #creates funciton to get the complete power array
    def getCompletePower(self):
        return self.Power_Array


    #creates function to get the norm power array
    def getNormPower(self):
        return self.normPower_Array
    
    #creates function to get the total energy
    def getTotalEnergy(self):
        return np.array([[self.totalEnergyOut]])
    
    def writeCurrent(self, filePath: str):
        df = pd.DataFrame(self.Current_Array)
        df.to_csv(filePath, header=False, index=False)  

    def writeVoltage(self, filePath: str):
        df = pd.DataFrame(self.Voltage_Array)
        df.to_csv(filePath, header=False, index=False)                

    #creates functions to write the power vectors to csv files
    def writeNormPower(self, filePath: str):
        df = pd.DataFrame(self.normPower_Array)
        df.to_csv(filePath, header=False, index=False)

    def writeCompletePower(self, filePath: str):
        df = pd.DataFrame(self.Power_Array)
        df.to_csv(filePath, header=False, index=False)

    def writeTotalEnergy(self, filePath: str):
        df = pd.DataFrame(self.totalEnergyOut)
        df.to_csv(filePath, header=False, index=False)



#class that writes down the deltas for the system
class deltaTracker:

    #creates the initialization function
    def __init__(self):

        #creates whole array to store the deltas
        self.deltaArray = np.ndarray((8,0))

    #creates the update function
    def update(self, delta: MsgDelta):

        temp = np.zeros((8,1))
        #extracts all the parts
        temp[0][0] = delta.throttle_0
        temp[1][0] = delta.throttle_1
        temp[2][0] = delta.throttle_2
        temp[3][0] = delta.throttle_3
        temp[4][0] = delta.throttle_4
        temp[5][0] = delta.elevator
        temp[6][0] = delta.aileron
        temp[7][0] = delta.rudder

        #concatenates the whole delta array
        self.deltaArray = np.concatenate((self.deltaArray, temp), axis=1)

    #getter
    def getDeltaArray(self):

        return self.deltaArray
    
    #writer
    def writeDeltaArray(self, filePath: str):

        df = pd.DataFrame(self.deltaArray)
        df.to_csv(filePath, header=False, index=False)




#class that writes down the state of the system and tracks it
class stateTracker:

    def __init__(self):

        self.totalState = np.ndarray((13,0))

        self.pos = np.ndarray((3,0))

        self.vel = np.ndarray((3,0))

        self.quat = np.ndarray((4,0))

        self.euler = np.ndarray((3,0))

        self.rates = np.ndarray((3,0))


    def update(self, state: np.ndarray):

        #reshapes the state
        state = np.reshape(state, (13,1))
        pos = np.reshape(state[0:3,:], (3,1))
        vel = np.reshape(state[3:6,:], (3,1))
        quat = np.reshape(state[6:10,:], (4,1))
        euler = np.reshape(Quaternion2Euler(quaternion=quat), (3,1))
        rates = np.reshape(state[10:13,:], (3,1))


        self.totalState = np.concatenate((self.totalState, state), axis=1)

        self.pos = np.concatenate((self.pos, pos), axis=1)

        self.vel = np.concatenate((self.vel, vel), axis=1)

        self.quat = np.concatenate((self.quat, quat), axis=1)

        self.euler = np.concatenate((self.euler, euler), axis=1)

        self.rates = np.concatenate((self.rates, rates), axis=1)


    def getTotalState(self):

        return self.totalState
    
    def getPos(self):

        return self.totalState

    def getVel(self):

        return self.totalState
    
    def getQuat(self):

        return self.totalState
    
    def getEuler(self):

        return self.totalState
    
    def getRates(self):

        return self.totalState
    


    def writeTotalState(self, filePath: str):

        df = pd.DataFrame(self.totalState)
        df.to_csv(filePath, header=False, index=False)
    
    def writePos(self, filePath: str):

        df = pd.DataFrame(self.pos)
        df.to_csv(filePath, header=False, index=False)

    def writeVel(self, filePath: str):

        df = pd.DataFrame(self.vel)
        df.to_csv(filePath, header=False, index=False)
    
    def writeQuat(self, filePath: str):

        df = pd.DataFrame(self.quat)
        df.to_csv(filePath, header=False, index=False)
    
    def writeEuler(self, filePath: str):

        df = pd.DataFrame(self.euler)
        df.to_csv(filePath, header=False, index=False)
    
    def writeRates(self, filePath: str):

        df = pd.DataFrame(self.rates)
        df.to_csv(filePath, header=False, index=False)

