#this file creates an enumeration to enumerate all the different sets of coefficients

#creates master class, which contains all the subclasses of aerodynamic coefficients

from enum import Enum

import numpy as np

import parameters.quad.anaconda_parameters as QUAD

#defines the enumeration for the specified coefficients sets
class ChosenAerodynamicSet(Enum):
    C_L_D = 0 #value corresponding to lift and drag coefficients
    C_Y = 1 #value corresponding to Y coefficients
    C_l = 2 #value corresponding to ell coefficients
    C_m = 3 #value corresponding to m coefficients
    C_n = 4 #value corresponding to n coefficients

#defines the 



##############################################################################################
#subsection with all of the smaller subclasses of aerodynamic coefficients

#creates the Lift and drag class
class MsgC_L_D:

    #creates the initialization function
    def __init__(self):
        #sets the chosen set
        self.chosenSet = ChosenAerodynamicSet.C_L_D
        #creates the initial coefficient of lift
        self.C_L_0 = 0.0
        #creates the coefficient of lift dependent on alpha
        self.C_L_alpha = 0.0
        #creates the coefficient of lift dependent on q
        self.C_L_q = 0.0
        #creates the coefficient of lift dependent on delta elevator
        self.C_L_delta_e = 0.0
        #creates the initial coefficient of drag
        self.C_D_0 = 0.0
        #creates the coefficient of drag dependent on alpha
        self.C_D_alpha = 0.0
        #creates the coefficient of drag dependent on q
        self.C_D_q = 0.0
        #creates the coefficient of drag dependent on delta elevator
        self.C_D_delta_e = 0.0


        #creates set of all coefficients
        self.coefficientSet = np.array([[self.C_L_0],
                                        [self.C_L_alpha],
                                        [self.C_L_q],
                                        [self.C_L_delta_e],
                                        [self.C_D_0],
                                        [self.C_D_alpha],
                                        [self.C_D_q],
                                        [self.C_D_delta_e]])
        
        #saves the set length
        self.setLength = 8

    #creates function to set the coefficients
    def setCoefficients(self, coefficients: np.array):

        #writes to the coefficient set
        self.coefficientSet = coefficients

    #creates function to get the coefficients
    def getCoefficients(self):
        return self.coefficientSet
    
    #creates function to get the set length
    def getSetLength(self):
        return self.setLength

#creates Y class
class MsgC_Y:
    def __init__(self):

        #sets the chosen set
        self.chosenSet = ChosenAerodynamicSet.C_Y
        #creates the different coefficients
        self.C_Y_0 = 0.0
        self.C_Y_beta = 0.0
        self.C_Y_p = 0.0
        self.C_Y_r = 0.0
        self.C_Y_delta_a = 0.0
        self.C_Y_delta_r = 0.0

        #creates the set of these
        #creates the compelte set
        self.coefficientSet = np.array([[self.C_Y_0],
                                        [self.C_Y_beta],
                                        [self.C_Y_p],
                                        [self.C_Y_r],
                                        [self.C_Y_delta_a],
                                        [self.C_Y_delta_r]])

        #saves the set length
        self.setLength = 6
        

    #creates function to set the coefficients
    def setCoefficients(self, coefficients: np.ndarray):

        #writes to the coefficient set
        self.coefficientSet = coefficients

    #creates function to get the coefficients
    def getCoefficients(self):
        return self.coefficientSet
    
    #creates function to get the set length
    def getSetLength(self):
        return self.setLength


#creates the l class
class MsgC_l:
    def __init__(self):
        #sets the chosen set
        self.chosenSet = ChosenAerodynamicSet.C_l

        #creates the different coefficients
        self.C_ell_0 = 0.0
        self.C_ell_beta = 0.0
        self.C_ell_p = 0.0
        self.C_ell_r = 0.0
        self.C_ell_delta_a = 0.0
        self.C_ell_delta_r = 0.0

        #creates the all set
        #creates the all set
        self.coefficientSet = np.array([[self.C_ell_0],
                                        [self.C_ell_beta],
                                        [self.C_ell_p],
                                        [self.C_ell_r],
                                        [self.C_ell_delta_a],
                                        [self.C_ell_delta_r]])
        
        #saves the set length
        self.setLength = 6
        

    #creates function to set the coefficients
    def setCoefficients(self, coefficients: np.array):

        #writes to the coefficient set
        self.coefficientSet = coefficients

    #creates function to get the coefficients
    def getCoefficients(self):
        return self.coefficientSet
    
    #gets the set length
    def getSetLength(self):
        return self.setLength
        
#creates the m class
class MsgC_m:
    def __init__(self):

        #sets the chosen set
        self.chosenSet = ChosenAerodynamicSet.C_m


        #creates the individual coefficients
        self.C_m_0 = 0.0
        self.C_m_alpha = 0.0
        self.C_m_q = 0.0
        self.C_m_delta_e = 0.0

        #creates the whole set
        self.coefficientSet = np.array([[self.C_m_0],
                                        [self.C_m_alpha],
                                        [self.C_m_q],
                                        [self.C_m_delta_e]])
        
        #saves the set length
        self.setLength = 4

    #creates function to set the coefficients
    def setCoefficients(self, coefficients: np.array):

        #writes to the coefficient set
        self.coefficientSet = coefficients

    #creates function to get the coefficients
    def getCoefficients(self):
        return self.coefficientSet

    #returns the set length
    def getSetLength(self):
        return self.setLength

#creates the n class
class MsgC_n:
    def __init__(self):

        #sets the chosen set
        self.chosenSet = ChosenAerodynamicSet.C_n


        #creates the individual coefficients
        self.C_n_0 = 0.0
        self.C_n_beta = 0.0
        self.C_n_p = 0.0
        self.C_n_r = 0.0
        self.C_n_delta_a = 0.0
        self.C_n_delta_r = 0.0

        #creates the whole set
        self.coefficientSet = np.array([[self.C_n_0],
                                        [self.C_n_beta],
                                        [self.C_n_p],
                                        [self.C_n_r],
                                        [self.C_n_delta_a],
                                        [self.C_n_delta_r]])
        
        #creates the set length
        self.setLength = 6


    #creates function to set the coefficients
    def setCoefficients(self, coefficients: np.array):

        #writes to the coefficient set
        self.coefficientSet = coefficients

    #creates function to get the coefficients
    def getCoefficients(self):
        return self.coefficientSet

    #gets the set length
    def getSetLength(self):
        return self.setLength

##############################################################################################


#creates the whole aerodynamics message, which is a list of the different aerodynamic coefficients
#that will be used in a particular maneuver
class MsgAerodynamics:

    #creates the init function
    def __init__(self):
        
        #creates array to store which type of aerodynamics message it actually is
        self.aerodynamicTypeList = []

        #creates the array to store all of the aerodynamic sets for the particular maneuver
        #and initializes the list to have zero as the first element, but we will overwrite this soon
        self.aerodynamicMessageList = []

        #gets the message list length
        self.aerodynamicMessageListLength = 0
        #creates the initialized flag
        self.initializedFlag = False

    #creates function to add an aerodynamic message to the list
    def addAerodynamicMessage(self, aerodynamicSubmessageType: ChosenAerodynamicSet):
        #appends the message type
        self.aerodynamicTypeList.append(aerodynamicSubmessageType)
        
        #appends an actual instance of the specified submessage class
        if aerodynamicSubmessageType == ChosenAerodynamicSet.C_L_D:
            self.aerodynamicMessageList.append(MsgC_L_D())
        elif aerodynamicSubmessageType == ChosenAerodynamicSet.C_Y:
            self.aerodynamicMessageList.append(MsgC_Y())
        elif aerodynamicSubmessageType == ChosenAerodynamicSet.C_l:
            self.aerodynamicMessageList.append(MsgC_l())
        elif aerodynamicSubmessageType == ChosenAerodynamicSet.C_m:
            self.aerodynamicMessageList.append(MsgC_m)
        elif aerodynamicSubmessageType == ChosenAerodynamicSet.C_n:
            self.aerodynamicMessageList.append(MsgC_n)

        #increments the aerodynamic message list length
        self.aerodynamicMessageListLength += 1

    #creates function to get an individual aerodynamic message from the list
    def getAerodynamicMessage(self, messageIndex: int):
        #returns that from the aerodynamics message list
        return self.aerodynamicMessageList[messageIndex]
    
    #creates function to get an individual aerodynamic message type from the list
    def getAerodynamicMessageType(self, messageIndex: int):
        #returns that from the aerodynamics type list
        return self.aerodynamicTypeList[messageIndex]


    #gets the length of the set list
    def getListLength(self):
        return len(self.aerodynamicMessageList)
    
