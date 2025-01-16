import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[4]))
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt

from tools.rotations import *


path = os.path.abspath("launch_files/trajectoryFollower/aircraftLineFollower/autopilotOutput")

#gets the euler angles
eulerAngles = pd.read_csv(path + '/eulerAnglesTrue.csv', header=None, index_col=False).values

#gets the forces and moments
forcesMomentsBodyFrame = pd.read_csv(path + '/forcesMomentsTrue.csv', header=None, index_col=False).values

#gets the desired forces and Moments
forcesDesired_singular = pd.read_csv(path + '/forcesDesired_singular.csv', header=None, index_col=False).values

#gets the desired forces and moments from the seperated controller
forcesDesired_seperated = pd.read_csv(path + '/forcesDesired_seperated.csv', header=None, index_col=False).values


#plots the two body frame x forces the actual forces experienced by the actual plane using
#the standard PID controller. the other the desired forces for each time step based on the state,
# which was recorded, but not used in actual controlling


plt.figure(0)
plt.plot(forcesMomentsBodyFrame[0,:], label='Fx Actual')
plt.plot(forcesDesired_singular[0,:], label='Fx Desired singular')
plt.plot(forcesDesired_seperated[0,:], label='Fx Desired seperated')
plt.legend()
plt.title('X Forces')
plt.show()

plt.figure(1)
plt.plot(forcesMomentsBodyFrame[1,:], label='Fy Actual')
plt.plot(forcesDesired_singular[1,:], label='Fy Desired singular')
plt.plot(forcesDesired_seperated[1,:], label='Fy Desired seperated')
plt.legend()
plt.title('Y Forces')
plt.show()


plt.figure(2)
plt.plot(forcesMomentsBodyFrame[2,:], label='Fz Actual')
plt.plot(forcesDesired_singular[2,:], label='Fz Desired singular')
plt.plot(forcesDesired_seperated[2,:], label='Fz Desired seperated')
plt.legend()
plt.title('Z Forces')
plt.show()


'''
#plots the body frame forces and moments
plt.figure(0)
plt.plot(forcesMomentsBodyFrame[0,:], label='Fx Body')
plt.plot(forcesMomentsBodyFrame[1,:], label='Fy Body')
plt.plot(forcesMomentsBodyFrame[2,:], label='Fz Body')
plt.legend()
plt.title("Forces Body")
plt.show()

plt.figure(1)
plt.plot(forcesMomentsBodyFrame[3,:], label='Mx Body')
plt.plot(forcesMomentsBodyFrame[4,:], label='My Body')
plt.plot(forcesMomentsBodyFrame[5,:], label='Mz Body')
plt.legend()
plt.title("Moments Body")
plt.show()


forcesInertialList = []

#iterates through and obtains the inertial frame forces and moments
numItems = np.shape(forcesMomentsBodyFrame)[1]
for i in range(numItems):
    currentForcesBody = forcesMomentsBodyFrame[0:3,i]
    #gets the current rotation matrix
    phi = eulerAngles[0,i]
    theta = eulerAngles[1,i]
    psi = eulerAngles[2,i]

    R_b2i = euler_to_rotation(phi, theta, psi)
    
    #obtains the current forces in inertial frame
    currentForcesInertial = R_b2i @ currentForcesBody
    #saves the forces in the inertial frame
    forcesInertialList.append(currentForcesInertial)

forcesInertialList = np.array(forcesInertialList).T

plt.figure(2)
plt.plot(forcesInertialList[0,:], label='Fx Inertial')
plt.plot(forcesInertialList[1,:], label='Fy Inertial')
plt.plot(forcesInertialList[2,:], label='Fz Inertial')
plt.legend()
plt.title("Forces Inertial")
plt.show()
#'''

potato = 0