'''
    Classes that implement differentiators
    
    DirtyDerivative:
        dirty derivative of a single signal
    RotationDerivative:
        differentiate a rotation matrix
'''
import numpy as np
from tools.lie_group import vee_SO3, log_SO3

class DirtyDerivative:
    '''
    Class that computes the time derivative of a signal using dirty derivative
    The transfer function is D(s)= s/(tau*s + 1)

    Attributes
    ----------
    _Ts : float
        sample rate
    _tau : float
        the cutoff frequency of is omega_co = 1/tau
    _y_dot : float
        current estimate of the derivative of y
    _y_delay_1 : float
        the signal y, delayed by one sample
    _a1 : float
        differentiator gain
    _a2 : float
        differentiator gain
    
    Methods
    -------
    update(y) :
        returns y_dot
    '''
    def __init__(self, 
                 Ts: float=0.01, 
                 tau: float=0.05,
                 ):
        self._Ts = Ts
        self._tau = tau
        self._y_dot = 0.0
        self._y_delay_1 = 0.0
        self._a1 = (2.0 * tau - Ts) / (2.0 * tau + Ts)
        self._a2 = 2.0 / (2.0 * tau + Ts)

    def update(self, 
               y: float,
               )->float:
        self._y_dot = self._a1 * self._y_dot + self._a2 * (y - self._y_delay_1)
        self._y_delta_1 = y
        return self._y_dot


class RotationDerivative:
    '''
    Class that computes the time derivative of a rotation matrix

    Attributes
    ----------
    _Ts : float
        sample rate
    _R_delta_1 : np.ndarray (3x3)
        the rotation matrix, delayed by one sample
    
    Methods
    -------
    update(R) :
        Given the differential equation: Rdot = R * \hat(omega)
        returns omega = vee(R^T * Rdot)
    '''
    def __init__(self, 
                 Ts: float=0.01, 
                 R0: np.ndarray=np.eye(3),
                 ):
        self._Ts = Ts
        self._R_delta_1 = R0

    def update(self, 
               R: np.ndarray,
               )->np.ndarray:
        omega = vee_SO3(log_SO3(self._R_delta_1.T @ R))/self._Ts
        self._R_delta_1 = R
        return omega



