import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))

#reads in the npz files
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from copy import copy
from tools.rotations import rotation_to_euler


stateArrayFile = np.load('/home/dben1182/Documents/vtolsim/launch_files/trajectoryFollower/outputFiles/state.npz')
stateTimeArray = stateArrayFile['arr_0']


#reats in the pitch free info
pitchFreeInfoFile = np.load('/home/dben1182/Documents/vtolsim/launch_files/trajectoryFollower/outputFiles/PitchFreeInfo.npz')
pos_err = pitchFreeInfoFile['arr_0']
vel_err = pitchFreeInfoFile['arr_1']
F_ds = pitchFreeInfoFile['arr_2']
R = pitchFreeInfoFile['arr_3']



#gets the number of samples
numSamples = len(stateTimeArray)

#gets the stateVectorLength
stateVectorLen = len(stateTimeArray[0])

#creates the temp n,e,d vector
pos_n_array = []
pos_e_array = []
pos_d_array = []
phi_array = []
theta_array = []
psi_array = []
for j in range(100):
    currentState = stateTimeArray[j]
    pos_n_array.append(copy(currentState.item(0)))
    pos_e_array.append(copy(currentState.item(1)))
    pos_d_array.append(copy(-currentState.item(2)))
    phi, theta, psi = rotation_to_euler(R=R[j])
    phi_array.append(copy(phi))
    theta_array.append(copy(theta))
    psi_array.append(copy(psi))

fig = plt.figure(0)
ax = fig.add_subplot(111, projection='3d')
ax.plot(pos_n_array, pos_e_array, pos_d_array)
plt.show()
plt.xlabel('x')
plt.ylabel('y')





plt.figure(1)
plt.plot(phi_array, label='phi')
plt.plot(theta_array, label='theta')
plt.plot(psi_array, label='psi')
plt.legend()
plt.show()







pine = 0