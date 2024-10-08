#this file implements aerodynamics estimation schemes for each of the sets of aerodynamics parameters

import numpy as np

import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))

#imports the needed message classes for our communications
from message_types.quad.msg_sensors import MsgSensors
from message_types.quad.msg_state import MsgState
from message_types.quad.msg_delta import MsgDelta

#imports the aerodynamics message class
from message_types.quad.msg_aerodynamics import MsgAerodynamics
from message_types.quad.msg_aerodynamics import ChosenAerodynamicSet
#imports the chosen aerodynamics enumeration
from message_types.quad.msg_aerodynamics import ChosenAerodynamicSet


#imports the stationary kalman filter class
from system_identification.stationaryKalmanFilter import stationaryKalmanFilter

#imports all of the different classes for the 5 sets
from message_types.quad.msg_aerodynamics import MsgC_L_D, MsgC_Y, MsgC_l, MsgC_m, MsgC_n


import parameters.quad.simulation_parameters as SIM

import parameters.quad.anaconda_parameters as QUAD
#parameters for the physical system
gravity = QUAD.gravity
#gets the surface area of the wing
S_wing = QUAD.S_wing
#gets b, the wingspan
b = QUAD.b
#gets c, the mean chord
c = QUAD.c
#gets the air density
rho = QUAD.rho
#gets the mass of the MAV
mass = QUAD.mass


#sets sigma
sigma = 0.0005


###################################################################################
#this section creates each of the 5 estimation classes: Y, Lift and Drag, l, m, and n
#these estimation classes are grouped together into the main aerodynamic estimator class,
#which can store multiple of the subclasses to do multiple estimations at once.

#creates estimation classes for each set of aerodynamic coefficients
class C_Y_Estimator:

    #creates the initialization function
    def __init__(self):

        #creates the mixing matrix
        self.mixingMatrix = 0

        #instantiates a Y coefficients message class
        self.MsgC_Y = MsgC_Y()

        #creates the coefficient set
        self.coefficientSet = 0
        #instantiates the kalman filter
        self.YKalmanFilter = stationaryKalmanFilter(x_length=self.MsgC_Y.getSetLength(), y_width=1)

    #creates the update function
    def update(self, sensorMeasurements: MsgSensors, state: MsgState, delta: MsgDelta, thrust):

        #gets the acceleration in the Y direction
        accel_y = sensorMeasurements.accel_y

        #gets the airspeed
        Va = state.Va

        #gets the scaling factor
        scalingFactor = ((1.0)/(2.0*mass))*rho*(Va**2)*S_wing 
        #
        #gets the a_n mevtor
        a_n = np.transpose(scalingFactor*self.getMixingMatrix(state=state, delta=delta))

        #gets the y_n
        y_n = np.array([[accel_y]])

        #gets the y coefficients by updating the kalman fulter
        self.coefficientSet = self.YKalmanFilter.update(y_n=y_n, a_n=a_n)
        #writes the message using the
        self.MsgC_Y.setCoefficients(coefficients=self.coefficientSet)
    
    #function to update and get the mixing matrix
    def getMixingMatrix(self, state: MsgState, delta: MsgDelta):

        #gets the beta
        beta = state.beta
        #gets the airspeed
        Va = state.Va
        #gets the roll rate
        p = (state.omega)[0][0]
        #gets the pitch rate
        q = (state.omega)[1][0]
        #gets the yaw rate
        r = (state.omega)[2][0]
        #gets the delta_a
        delta_a = delta.aileron
        #gets the delta_r
        delta_r = delta.rudder

        ##creates the mixing matrix
        self.mixingMatrix = np.array([[1.0, beta, ((b)/(2*Va))*p, ((b)/(2*Va))*r, delta_a, delta_r]])

        #returns the mixing matrix
        return self.mixingMatrix

    #function that returns the submessage
    def getMessage(self):
        return self.MsgC_Y


#creates the Lift and Drag Coefficient estimation class
class C_L_D_Estimator:

    #creates the initialization function
    def __init__(self):
        #creates the mixing matrix
        self.mixingMatrix = 0

        #instantiates the lift drag coefficients message class
        self.MsgC_L_D = MsgC_L_D()

        #creates the coefficient set
        self.coefficientSet = 0

        #instantiates the kalman filter
        self.L_D_KalmanFilter = stationaryKalmanFilter(x_length=self.MsgC_L_D.getSetLength(), y_width=2)

    #creates the update function
    def update(self, sensorMeasurements: MsgSensors, state: MsgState, delta: MsgDelta, thrust):
        
        #gets the accel x
        accel_x = sensorMeasurements.accel_x

        #gets the accel z
        accel_z = sensorMeasurements.accel_z

        #gets the airspeed
        Va = state.Va

        #gets the scaling factor
        scalingFactor = ((-1.0)/(2.0*mass))*rho*(Va**2)*S_wing

        #gets the a_n vector
        a_n = np.transpose(scalingFactor*self.getMixingMatrix(state=state, delta=delta))

        #gets the y_n array as accelerations minus the thrust of the propeller
        y_n = np.array([[accel_x], [accel_z]]) - np.array([[thrust/mass], [0.0]])        

        #gets the coefficient set
        self.coefficientSet = self.L_D_KalmanFilter.update(y_n=y_n, a_n=a_n)
        #writes that to the aerodynamic message
        self.MsgC_L_D.setCoefficients(coefficients=self.coefficientSet)

    #creates the helpef function to create the mixing matrix
    def getMixingMatrix(self, state: MsgState, delta: MsgDelta):

        #gets alpha, the angle of attach
        alpha = state.alpha
        #gets the roll rate
        p = (state.omega)[0][0]
        #gets the pitch rate
        q = (state.omega)[1][0]
        #gets the yaw rate
        r = (state.omega)[2][0]
        #gets the airspeed
        Va = state.Va
        #gets the elevator's delta
        delta_e = delta.elevator


        #gets the alpha rotation matrix
        alphaRotationMatrix = np.array([[-np.sin(alpha), np.cos(alpha)],
                                        [np.cos(alpha), np.sin(alpha)]])
        
        #gets the primary mixing matrix
        primaryMixingMatrix = np.array([[1.0, alpha, (c*q)/(2.0*Va), delta_e, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 1.0, alpha, (c*q)/(2.0*Va), delta_e]])
        
        #gets the whole mixing matrix
        self.mixingMatrix = alphaRotationMatrix @ primaryMixingMatrix

        #returns the mixing matrix
        return self.mixingMatrix


    #gets the LD aerodynamic message
    def getCoefficientMessage(self):
        return self.MsgC_L_D

    #function that returns the submessage
    def getMessage(self):
        return self.MsgC_L_D

#creates the l coefficient estimation class
class C_l_Estimator:
    #creates the init function
    def __init__(self):
        #creates the mixing matrix
        self.mixingMatrix = 0 

        #instantiates the l coefficients message class
        self.MsgC_l = MsgC_l()

        #coefficient set
        self.coefficientSet = 0
        #instantiates the kalman filter
        self.l_KalmanFilter = stationaryKalmanFilter(x_length=self.MsgC_l.getSetLength(), y_width=1)

        #instantiates the three dirty derivative functions
        #creates dirty derivative function for p
        self.pDirtyDerivative = DirtyDerivative(Ts=SIM.ts_control, sigma=sigma)
        #creates the dirty derivative function for q
        self.qDirtyDerivative = DirtyDerivative(Ts=SIM.ts_control, sigma=sigma)
        #creates the dirty derivative function for r
        self.rDirtyDerivative = DirtyDerivative(Ts=SIM.ts_control, sigma=sigma)

    #creates the update function
    def update(self, sensorMeasurements: MsgSensors, state: MsgState, delta: MsgDelta, thrust):

        #uses p, q, and r from the estimator, and not the direct gyro measurements.
        #we do this because the estimator already low pass filters the gyro measurements,
        #and subtracts the estimated bias, which gives a much more accurate estimate for everything
        #gets the roll rate
        p = (state.omega)[0][0]
        #gets the pitch rate
        q = (state.omega)[1][0]
        #gets the yaw rate
        r = (state.omega)[2][0]
        #calls the three dirty derivative functions to get p_dot, q_dot, and r_dot
        p_dot = self.pDirtyDerivative.update(y=p)
        #calls for q_dot
        q_dot = self.qDirtyDerivative.update(y=q)
        #calls for r_dot
        r_dot = self.rDirtyDerivative.update(y=r)

        #gets the Js pertinant for this calculation
        Jx = QUAD.Jx
        Jy = QUAD.Jy
        Jz = QUAD.Jz
        Jxz = QUAD.Jxz        


        #gets the airspeed Va
        Va = state.Va

        #gets the air density rho
        rho = QUAD.rho
        #gets the surface area of the wing
        S = QUAD.S_wing
        #gets the b
        b = QUAD.b

        #gets the scaling factor
        scalingFactor = ((1.0)/2.0)*rho*(Va**2)*S*b

        #gets the a_n vector
        a_n = np.transpose(scalingFactor*self.getMixingMatrix(state, delta))

        #gets the y_n
        y_n = np.array([[Jx*p_dot - Jy*q*r + Jz*q*r - Jxz*r_dot - Jxz*p*q]])

        #updates the coefficient set
        self.coefficientSet = self.l_KalmanFilter.update(y_n, a_n)
        #saves those coefficients to the message class
        self.MsgC_l.setCoefficients(coefficients=self.coefficientSet)


    #helper function to get the mixing matrix
    def getMixingMatrix(self, state: MsgState, delta: MsgDelta):
        
        #gets the beta
        beta = state.beta
        #gets the airspeed
        Va = state.Va
        #gets the roll rate
        p = (state.omega)[0][0]
        #gets the pitch rate
        q = (state.omega)[1][0]
        #gets the yaw rate
        r = (state.omega)[2][0]
        #gets the aileron delta a
        delta_a = delta.aileron
        #gets the delta_r
        delta_r = delta.rudder

        #gets the b
        b = QUAD.b

        #case all set
        self.mixingMatrix = np.array([[1.0, beta, (b*p)/(2*Va), (b*r)/(2*Va), delta_a, delta_r]])
        
        #returns the mixing matrix
        return self.mixingMatrix

    #function that returns the submessage
    def getMessage(self):
        return self.MsgC_l

#creates the m coefficient estimation class
class C_m_Estimator:

    #creates the initialization function
    def __init__(self):

        #creates the mixing matrix
        self.mixingMatrix = 0

        #instantiates a Msg_M class
        self.MsgC_m = MsgC_m()

        #instantiates the kalman filter
        self.mKalmanFilter = stationaryKalmanFilter(x_length=self.MsgC_m.getSetLength(), y_width=1)

        #instantiates the three dirty derivative functions
        #creates dirty derivative function for p
        self.pDirtyDerivative = DirtyDerivative(Ts=SIM.ts_simulation, sigma=sigma)
        #creates the dirty derivative function for q
        self.qDirtyDerivative = DirtyDerivative(Ts=SIM.ts_simulation, sigma=sigma)
        #creates the dirty derivative function for r
        self.rDirtyDerivative = DirtyDerivative(Ts=SIM.ts_simulation, sigma=sigma)


    #creates the update function, which updates the estimate for the aerodynamic coefficients at each step
    def update(self, sensorMeasurements: MsgSensors, state: MsgState, delta: MsgDelta, thrust):
        #saves the J_x constant
        Jx = QUAD.Jx
        #saves the Jy constant
        Jy = QUAD.Jy
        #saves the Jz constant
        Jz = QUAD.Jz
        #saves the Jxz constant
        Jxz = QUAD.Jxz

        #saves the air density
        rho = QUAD.rho

        #saves the airspeed
        Va = state.Va

        #saves the surface area
        S = QUAD.S_wing

        #saves c
        c = QUAD.c

        #gets the roll rate
        p = (state.omega)[0][0]
        #gets the pitch rate
        q = (state.omega)[1][0]
        #gets the yaw rate
        r = (state.omega)[2][0]

        #gets the derivative of p, q, and r
        p_dot = self.pDirtyDerivative.update(y=p)
        q_dot = self.qDirtyDerivative.update(y=q)
        r_dot = self.rDirtyDerivative.update(y=r)

        #creates the scaling factor
        scalingFactor = (1/2)*rho*(Va**2)*S*c

        #gets the a_n
        a_n = np.transpose(scalingFactor*self.getMixingMatrix(state, delta))

        #creates the y_n
        y_n = np.array([[Jy*q_dot + p*(Jx*r + Jxz*p) - r*(Jz*p + Jxz*r)]])

        #gets the coefficient set
        coefficientSet = self.mKalmanFilter.update(y_n=y_n, a_n=a_n)

        #sets it to the message
        self.MsgC_m.setCoefficients(coefficients=coefficientSet)

    #gets the mixing matrix
    def getMixingMatrix(self, state: MsgState, delta: MsgDelta):

        #gets the alpha
        alpha = state.alpha

        #gets the roll rate
        p = (state.omega)[0][0]
        #gets the pitch rate
        q = (state.omega)[1][0]
        #gets the yaw rate
        r = (state.omega)[2][0]
        #gets the delta e
        delta_e = delta.elevator

        #gets the airspeed
        Va = state.Va

        #gets the c
        c = QUAD.c

        #case all set
        self.mixingMatrix = np.array([[1.0, alpha, ((c*q)/(2.0*Va)), delta_e]])

        #returns the mixing matrix
        return self.mixingMatrix

    #function that returns the submessage
    def getMessage(self):
        return self.MsgC_m

#creates the n coefficient estimation class
class C_n_Estimator:

    #creates the init function
    def __init__(self):

        #creates the mixing matrix
        self.mixingMatrix = 0

        #instantiates the Msg_n class
        self.MsgC_n = MsgC_n()

        #instantiates the kalman filter
        self.nKalmanFilter = stationaryKalmanFilter(x_length=self.MsgC_n.getSetLength(), y_width=1)

        #instantiates the three dirty derivative functions
        #creates dirty derivative function for p
        self.pDirtyDerivative = DirtyDerivative(Ts=SIM.ts_simulation, sigma=sigma)
        #creates the dirty derivative function for q
        self.qDirtyDerivative = DirtyDerivative(Ts=SIM.ts_simulation, sigma=sigma)
        #creates the dirty derivative function for r
        self.rDirtyDerivative = DirtyDerivative(Ts=SIM.ts_simulation, sigma=sigma)     


    #update function
    def update(self, sensorMeasurements: MsgSensors, state: MsgState, delta: MsgDelta, thrust):

        #saves the Jx, Jy, Jz, and Jxz constants
        Jx = QUAD.Jx
        Jy = QUAD.Jy
        Jz = QUAD.Jz
        Jxz = QUAD.Jxz

        #saves the air density
        rho = QUAD.rho

        #saves the airspeed
        Va = state.Va

        #saves the wing surface area
        S = QUAD.S_wing

        #saves b
        b = QUAD.b

        #gets the roll rate
        p = (state.omega)[0][0]
        #gets the pitch rate
        q = (state.omega)[1][0]
        #gets the yaw rate
        r = (state.omega)[2][0]

        #gets the p_dot, q_dot, and r_dot
        p_dot = self.pDirtyDerivative.update(y=p)
        q_dot = self.qDirtyDerivative.update(y=q)
        r_dot = self.rDirtyDerivative.update(y=r)

        #creates the scaling factor
        scalingFactor = (1/2)*rho*(Va**2)*S*b

        #gets the a_n
        a_n = np.transpose(scalingFactor*self.getMixingMatrix(state=state, delta=delta))

        #creates the y_n
        y_n = np.array([[-Jx*p*q + Jy*p*q + Jz*r_dot - Jxz*p_dot + Jxz*q*r]])

        #gets the coefficients
        coefficientSet = self.nKalmanFilter.update(y_n, a_n)

        #writes it to the message
        self.MsgC_n.setCoefficients(coefficients=coefficientSet)

    #gets mixing matrix
    def getMixingMatrix(self, state: MsgState, delta: MsgDelta):


        #saves the beta
        beta = state.beta


        #gets the roll rate
        p = (state.omega)[0][0]
        #gets the pitch rate
        q = (state.omega)[1][0]
        #gets the yaw rate
        r = (state.omega)[2][0]

        #delta a
        delta_a = delta.aileron

        #saves delta r
        delta_r = delta.rudder

        #saves the airspeed
        Va = state.Va

        #gets the b
        b = QUAD.b

        #creates the mixing matrix
        self.mixingMatrix = np.array([[1.0, beta, ((b*p)/(2*Va)), ((b*r)/(2*Va)), delta_a, delta_r]])

        #returns it 
        return self.mixingMatrix

    #function that returns the submessage
    def getMessage(self):
        return self.MsgC_n
    

#creates the Aerodynamics Estimator master class
class AerodynamicsEstimator:

    #defines the initialization funciton
    def __init__(self, aerodynamicMessage: MsgAerodynamics):
        #saves the aerodynamic message
        self.aerodynamicMessage = aerodynamicMessage

        #creates the list that contains all of the aerodynamics estimators
        self.aerodynamicsEstimatorList = [0]
        initialized = False

        #iterates through the number of estimators
        for i in range(self.aerodynamicMessage.getListLength()):
            
            #gets the message type
            messageType = self.aerodynamicMessage.getAerodynamicMessageType(messageIndex=i)

            #case Y
            if messageType == ChosenAerodynamicSet.C_Y:
                estimator = C_Y_Estimator()
            #case Lift and Drag Coefficients
            elif messageType == ChosenAerodynamicSet.C_L_D:
                estimator = C_L_D_Estimator()
            #case l coefficients
            elif messageType == ChosenAerodynamicSet.C_l:
                estimator = C_l_Estimator()
            #case m coefficients
            elif messageType == ChosenAerodynamicSet.C_m:
                estimator = C_m_Estimator()
            #case n coefficients
            elif messageType == ChosenAerodynamicSet.C_n:
                estimator = C_n_Estimator()


            #case initialized
            if initialized:
                self.aerodynamicsEstimatorList.append(estimator)
            #case uninitialized
            else:
                self.aerodynamicsEstimatorList[0] = estimator
                #sets to initialized
                initialized = True

    #creates the update function for the estimator list
    def update(self, sensorMeasurements: MsgSensors, state: MsgState, delta: MsgDelta, thrust):

        #iterates through and updates all of the sub estimators
        for i in range(self.aerodynamicMessage.getListLength()):
            self.aerodynamicsEstimatorList[i].update(sensorMeasurements, state, delta, thrust)

    #gets the coefficient message classes from each of the sub estimators, and 
    def getCoefficients(self):

        messageList = []
        for i in range(self.aerodynamicMessage.getListLength()):
            #case initialized
            messageList.append(self.aerodynamicsEstimatorList[i].getMessage())

        return messageList


#creates the dirty derivative class
class DirtyDerivative:
    # return the dirty derivative of signal y
    def __init__(self, Ts, sigma=0.05):
        beta = 0.9
        self.a1 = ((2.0*sigma - Ts)/(2.0*sigma + Ts))
        self.a2 = ((2.0)/(2.0*sigma + Ts))
        self.y_dot = 0.0
        self.y_delay_1 = 0.0
        self.initialized = False
    
    #creates the update function
    def update(self, y):
        #case initialized already
        if self.initialized:
            self.y_dot = self.a1 * self.y_dot + self.a2 * (y - self.y_delay_1)
        #case needs initialization
        else:
            self.y_dot = 0.0
            self.initialized = True
        #saves the current y as the future delayed y
        self.y_delay_1 = y
        #returns the y dot
        return self.y_dot