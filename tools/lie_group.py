"""
  some tools for working with Lie groups
"""
import numpy as np

def antisymmetric(mat: np.ndarray)->np.ndarray:
    '''
    returns the antisymmetric part of a matrix
    '''
    return (1./2.)*(mat - mat.T)


def log_SO3(R: np.ndarray)->np.ndarray:
    '''
        Log of a matrix in SO(3)
    '''
    theta = np.arccos(0.5 * (np.trace(R)-1))
    S = 0.5 * (R-R.T) / np.sinc(theta)
    return S


def hat_SO3(omega: np.ndarray)->np.ndarray:
    '''
        hat map for SO(3)
        maps 3x1 vector to 3x3 skew symmetric matrix
    '''
    S = np.array([
        [0., -omega.item(2), omega.item(1)],
        [omega.item(2), 0., -omega.item(0)],
        [-omega.item(1), omega.item(0), 0.],
        ])
    return S


def vee_SO3(S: np.ndarray)->np.ndarray:
    '''
        vee map for SO(3)
        maps 3x3 skew symmetric matrix to 3x1 vector
    '''
    if np.linalg.norm(S+S.T)!=0:
        disp('vee:  S must be skew symmetric')
    else:
        omega = np.array([[S[2,1]],[S[0,3]],[S[1,0]]])
    return omega
