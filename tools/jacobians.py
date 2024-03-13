import numpy as np
from typing import Callable


def jacobian(func: Callable, x: np.ndarray) -> np.ndarray:
    '''
        Compute jacobian of func(x) with respect to x
            f: R^n -> R^m
        Parameters:
            x: numpy ndarray (n x 1)  
        Returns:
            J: numpy ndarray (m x n)
    '''
    f = func(x)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.0001  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = func(x_eps)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J


def jacobian_x(func: Callable, x: np.ndarray, u: np.ndarray) -> np.ndarray:
    '''
        Compute jacobian of func(x,u) with respect to x
            f: R^n x R^p -> R^m
        Parameters:
            x: numpy ndarray (n x 1) 
            u: numpy ndarray (p x 1) 
        Returns:
            J: numpy ndarray (m x n)
    '''
    f = func(x, u)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.0001  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = func(x_eps, u)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J


def jacobian_u(func: Callable, x: np.ndarray, u: np.ndarray) -> np.ndarray:
    '''
        Compute jacobian of func(x,u) with respect to u
            f: R^n x R^p -> R^m
        Parameters:
            x: numpy ndarray (n x 1) 
            u: numpy ndarray (p x 1) 
        Returns:
            J: numpy ndarray (m x p)
    '''
    f = func(x, u)
    m = f.shape[0]
    n = u.shape[0]
    eps = 0.0001  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        u_eps = np.copy(u)
        u_eps[i][0] += eps
        f_eps = func(x, u_eps)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J