#this file implements the message class for the aerodynamic coefficients

from enum import Enum
import numpy as np

import parameters.quad.anaconda_parameters as QUAD

#creates the aerodynamic message class
class MsgAerodynamics:

    #creates the initialization function
    def __init__(self):

        #Lift Drag Coefficients
        self.C_L_0 = 0.0
        self.C_L_alpha = 0.0
        self.C_L_q = 0.0
        self.C_L_delta_e = 0.0
        self.C_D_0 = 0.0
        self.C_D_alpha = 0.0
        self.C_D_q = 0.0
        self.C_D_delta_e = 0.0

        self.L_D_Coef = np.array([[self.C_L_0],
                                  [self.C_L_alpha],
                                  [self.C_L_q],
                                  [self.C_L_delta_e],
                                  [self.C_D_0],
                                  [self.C_D_alpha],
                                  [self.C_D_q],
                                  [self.C_D_delta_e]])

        #Y Coefficients
        self.C_Y_0 = 0.0
        self.C_Y_beta = 0.0
        self.C_Y_p = 0.0
        self.C_Y_r = 0.0
        self.C_Y_delta_a = 0.0
        self.C_Y_delta_r = 0.0

        self.Y_Coef = np.array([[self.C_Y_0],
                                [self.C_Y_beta],
                                [self.C_Y_p],
                                [self.C_Y_r],
                                [self.C_Y_delta_a],
                                [self.C_Y_delta_r]])
        
        #ell Coefficients
        self.C_ell_0 = 0.0
        self.C_ell_beta = 0.0
        self.C_ell_p = 0.0
        self.C_ell_r = 0.0
        self.C_ell_delta_a = 0.0
        self.C_ell_delta_r = 0.0

        self.ell_Coef = np.array([[self.C_ell_0],
                                  [self.C_ell_beta],
                                  [self.C_ell_p],
                                  [self.C_ell_r],
                                  [self.C_ell_delta_a],
                                  [self.C_ell_delta_r]])

        #m coefficients
        self.C_m_0 = 0.0
        self.C_m_alpha = 0.0
        self.C_m_q = 0.0
        self.C_m_delta_e = 0.0

        self.m_Coef = np.array([[self.C_m_0],
                                [self.C_m_alpha],
                                [self.C_m_q],
                                [self.C_m_delta_e]])

        #n coefficients
        self.C_n_0 = 0.0
        self.C_n_beta = 0.0
        self.C_n_p = 0.0
        self.C_n_r = 0.0
        self.C_n_delta_a = 0.0
        self.C_n_delta_r = 0.0

        self.n_Coef = np.array([[self.C_n_0],
                                [self.C_n_beta],
                                [self.C_n_p],
                                [self.C_n_r],
                                [self.C_n_delta_a],
                                [self.C_n_delta_r]])



    #creates the setter function
    def setLDCoef(self, coefficients: np.ndarray):
        self.C_L_0 = coefficients.item(0)
        self.C_L_alpha = coefficients.item(1)
        self.C_L_Q = coefficients.item(2)
        self.C_L_delta_e = coefficients.item(3)
        
        
        self.C_D_0 = coefficients.item(4)
        self.C_D_alpha = coefficients.item(5)
        self.C_D_Q = coefficients.item(6)
        self.C_D_delta_e = coefficients.item(7)
        
        #calls the synchronize arrays
        self.synchronizeArrays()

    def setYCoef(self, coefficients: np.ndarray):
        self.C_Y_0 = coefficients.item(0)
        self.C_Y_beta = coefficients.item(1)
        self.C_Y_p = coefficients.item(2)
        self.C_Y_r = coefficients.item(3)
        self.C_Y_delta_a = coefficients.item(4)
        self.C_Y_delta_r = coefficients.item(5)

        #calls the synchronize arrays
        self.synchronizeArrays()


    def setEllCoef(self, coefficients: np.ndarray):
        self.C_ell_0 = coefficients.item(0)
        self.C_ell_beta = coefficients.item(1)
        self.C_ell_p = coefficients.item(2)
        self.C_ell_r = coefficients.item(3)
        self.C_ell_delta_a = coefficients.item(4)
        self.C_ell_delta_r = coefficients.item(5)

        #calls the synchronize arrays
        self.synchronizeArrays()

    def setMCoef(self, coefficients: np.ndarray):
        self.C_m_0 = coefficients.item(0)
        self.C_m_alpha = coefficients.item(1)
        self.C_m_q = coefficients.item(2)
        self.C_m_delta_e = coefficients.item(3)

        #calls the synchronize arrays
        self.synchronizeArrays()


    def setNCoef(self, coefficients: np.ndarray):
        self.C_n_0 = coefficients.item(0)
        self.C_n_beta = coefficients.item(1)
        self.C_n_p = coefficients.item(2)
        self.C_n_r = coefficients.item(3)
        self.C_n_delta_a = coefficients.item(4)
        self.C_n_delta_r = coefficients.item(5)

        #calls the synchronize arrays
        self.synchronizeArrays()


    #function to synchronize the variables to the arrays
    def synchronizeArrays(self):
        #sets the lift and drag coefficients
        self.L_D_Coef = np.array([[self.C_L_0],
                                  [self.C_L_alpha],
                                  [self.C_L_q],
                                  [self.C_L_delta_e],
                                  [self.C_D_0],
                                  [self.C_D_alpha],
                                  [self.C_D_q],
                                  [self.C_D_delta_e]])
        #sets the Y coefficients
        self.Y_Coef = np.array([[self.C_Y_0],
                                [self.C_Y_beta],
                                [self.C_Y_p],
                                [self.C_Y_r],
                                [self.C_Y_delta_a],
                                [self.C_Y_delta_r]])
        #sets the ell coefficients
        self.ell_Coef = np.array([[self.C_ell_0],
                                  [self.C_ell_beta],
                                  [self.C_ell_p],
                                  [self.C_ell_r],
                                  [self.C_ell_delta_a],
                                  [self.C_ell_delta_r]])
        #sets the m coefficients
        self.m_Coef = np.array([[self.C_m_0],
                                [self.C_m_alpha],
                                [self.C_m_q],
                                [self.C_m_delta_e]])
        #sets the n coefficients
        self.n_Coef = np.array([[self.C_n_0],
                                [self.C_n_beta],
                                [self.C_n_p],
                                [self.C_n_r],
                                [self.C_n_delta_a],
                                [self.C_n_delta_r]])



    #creates the getter functions

    def getLDCoef(self):
        array = np.array([[self.C_L_0],
                           [self.C_L_alpha],
                           [self.C_L_q],
                           [self.C_L_delta_e],
                           [self.C_D_0],
                           [self.C_D_alpha],
                           [self.C_D_q],
                           [self.C_D_delta_e]])
        
        return array
    
    def getYCoef(self):
        array = np.array([[self.C_Y_0],
                          [self.C_Y_beta],
                          [self.C_Y_p],
                          [self.C_Y_r],
                          [self.C_Y_delta_a],
                          [self.C_Y_delta_r]])
        
        return array
    
    def getEllCoef(self):
        array = np.array([[self.C_ell_0],
                          [self.C_ell_beta],
                          [self.C_ell_p],
                          [self.C_ell_r],
                          [self.C_ell_delta_a],
                          [self.C_ell_delta_r]])
        
        return array
    
    def getMCoef(self):
        array = np.array([[self.C_m_0],
                          [self.C_m_alpha],
                          [self.C_m_q],
                          [self.C_m_delta_e]])
        return array

    def getNCoef(self):
        array = np.array([[self.C_n_0],
                          [self.C_n_beta],
                          [self.C_n_p],
                          [self.C_n_r],
                          [self.C_n_delta_a],
                          [self.C_n_delta_r]])        
        
        return array

    
