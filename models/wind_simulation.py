"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
from tools.transfer_function import TransferFunction
import numpy as np
import parameters.anaconda_parameters as SIM


class WindSimulation:
    def __init__(self, Ts: float):
        '''
        steady state wind defined in the inertial frame
        '''
        self._steady_state = np.array([[0., 0., 0.]]).T
        self._steady_state = np.array([[0., 1., 0.]]).T

        #   Dryden gust model parameters (pg 56 UAV book)
        #Va = 25 # must set Va to a constant value
        Va = SIM.Va0
        #
        Lu = 200.0
        Lv = 200.0
        Lw = 50.0
        gust_flag = True
        if gust_flag==True:
            sigma_u = 1.06
            sigma_v = 1.06
            sigma_w = 0.7
        else:
            sigma_u = 0.0
            sigma_v = 0.0
            sigma_w = 0.0
        a1 = sigma_u*np.sqrt(2.*Va/(np.pi*Lu))
        a2 = sigma_v*np.sqrt(3.*Va/(np.pi*Lv))
        a3 = a2*Va/np.sqrt(3)/Lv
        a4 = sigma_w*np.sqrt(3.*Va/(np.pi*Lw))
        a5 = a4*Va/np.sqrt(3)/Lw
        b1 = Va/Lu
        b2 = Va/Lv
        b3 = Va/Lw
        self.u_w = TransferFunction(num=np.array([[a1]]),
                                     den=np.array([[1, b1]]),
                                     Ts=Ts)
        self.v_w = TransferFunction(num=np.array([[a2, a3]]),
                                     den=np.array([[1, 2*b2, b2**2.0]]),
                                     Ts=Ts)
        self.w_w = TransferFunction(num=np.array([[a4, a5]]),
                                     den=np.array([[1, 2*b3, b3**2.0]]),
                                     Ts=Ts)
        self._Ts = Ts

    def update(self) -> np.ndarray:
        ''' returns a six vector.
            The first three elements are the steady state wind in the inertial frame
            The second three elements are the gust in the body frame 
        '''
        gust = np.array([[self.u_w.update(np.random.randn())],
                         [self.v_w.update(np.random.randn())],
                         [self.w_w.update(np.random.randn())]])
        gust = np.array([[0.],[0.],[0.]])
        return np.concatenate(( self._steady_state, gust ))

