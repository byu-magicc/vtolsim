import numpy as np

#this file implements a message class for all of the coefficients of aerodynamics for 
#this particular system

class MsgCoefficients:

    #creates the initialization function
    def __init__(self):

        #defines all of the L coefficients
        self.C_L_0 = 0.0
        self.C_L_alpha = 0.0
        self.C_L_q = 0.0
        self.C_L_delta_e = 0.0
        self.C_D_0 = 0.0
        self.C_D_alpha = 0.0
        self.C_D_q = 0.0
        self.C_D_delta_e = 0.0

        #defines the Y coefficients
        self.C_Y_0 = 0.0
        self.C_Y_beta = 0.0
        self.C_Y_p = 0.0
        self.C_Y_r = 0.0
        self.C_Y_delta_a = 0.0
        self.C_Y_delta_r = 0.0

        #defines the ell coefficients
        self.C_ell_0 = 0.0
        self.C_ell_beta = 0.0
        self.C_ell_p = 0.0
        self.C_ell_r = 0.0
        self.C_ell_delta_a = 0.0
        self.C_ell_delta_r = 0.0

        #defines the m coefficients
        self.C_m_0 = 0.0
        self.C_m_alpha = 0.0
        self.C_m_q = 0.0
        self.C_m_delta_e = 0.0

        #defines the n coefficients
        self.C_n_0 = 0.0
        self.C_n_beta = 0.0
        self.C_n_p = 0.0
        self.C_n_r = 0.0
        self.C_n_delta_a = 0.0
        self.C_n_delta_r = 0.0


    #creates function to get Lift drag subset
    def setLiftDrag(self, LiftDragCoefficients: np.ndarray):
        self.C_L_0 = LiftDragCoefficients.item(0)
        self.C_L_alpha = LiftDragCoefficients.item(1)
        self.C_L_q = LiftDragCoefficients.item(2)
        self.C_L_delta_e = LiftDragCoefficients.item(3)
        self.C_D_0 = LiftDragCoefficients.item(4)
        self.C_D_alpha = LiftDragCoefficients.item(5)
        self.C_D_q = LiftDragCoefficients.item(6)
        self.C_D_delta_e = LiftDragCoefficients.item(7)

    #creates the get lift drag
    def getLiftDrag(self):
        liftDragArray = np.array([[self.C_L_0],
                                  [self.C_L_alpha],
                                  [self.C_L_q],
                                  [self.C_L_delta_e],
                                  [self.C_D_0],
                                  [self.C_D_alpha],
                                  [self.C_D_q],
                                  [self.C_D_delta_e]])
        
        return liftDragArray
    
    