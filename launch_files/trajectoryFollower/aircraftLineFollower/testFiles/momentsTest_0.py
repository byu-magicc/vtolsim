#This file implements the test file which tests whether the the desired Moments funciton is operating correctly.

import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[4]))
import numpy as np
import pandas as pd


from trajectory.trajectory_tracker import TrajectoryTracker
import parameters.anaconda_parameters as QUAD

from spatialmath.base import skew, skewa
from tools.rotations import *

import matplotlib.pyplot as plt



# we will simply be using the euler to rotation tool, setting psi and theta to zero

#phi will start as zero, and then have a constant angular acceleration, which should correspond
#to a moment about the x axis.

#creates the time linspace

startTime = 0.0
endTime = 10.0
numSamples = 1000
t = np.linspace(start=startTime, stop=endTime, num=numSamples)

#gets the J matrix
J = QUAD.J


#sets the acceleration constant to one for the roll, and zero for pitch and yaw
alpha = np.array([[1.0],
                  [0.0],
                  [0.0]])

#sets the omega_0
omega_0 = np.array([[0.0],
                    [0.0],
                    [0.0]])

#sets the theta_0, which encompasses roll, pitch, and yaw inclusive
theta_0 = np.array([[0.0],
                    [0.0],
                    [0.0]])

#gets the omega vector as a function of time. 
omega = alpha*t + omega_0

#gets the theta vector
theta = 0.5*alpha*(t**2) + omega_0*t + theta_0

#gets the actual moments of the system
momentsActual = []

for i in range(numSamples):
    #gets  the first temporary part
    temp1 = J @ alpha

    #gets the second temporary part
    temp2 = skew(omega[:,i]) @ J @ omega[:,i].reshape((3,1))

    #appends the sum
    momentsActual.append(temp1 + temp2)



#converts the moments actual into a vector
momentsActual = np.array(momentsActual)[:,:,0].T



#instantiates the trajectory tracker
traj_tracker = TrajectoryTracker(K_p = np.eye(3), K_d = np.eye(3))


#now, it is time to use the above data to create the rotation matrices,
#and then to compare the output moments
#creates the array for the moments calculated
momentsCalculated = []

#creates the rotation matrices list
rotationMatrices = []

for i in range(numSamples):
    #gets the rotation from the theta vector
    currentTheta = theta[:,i]
    currentRotation = euler_to_rotation(phi=currentTheta.item(0),
                                        theta=currentTheta.item(1),
                                        psi=currentTheta.item(2))

    rotationMatrices.append(currentRotation)
    
    #gets the moment using the trajectory tracker class
    calculatedMoment = traj_tracker.getMomentsDesired(R_desired_inertial=currentRotation)

    #appends to the momentsCalculated array
    momentsCalculated.append(calculatedMoment)


#reshapes the moments calculated vector
momentsCalculated = np.array(momentsCalculated)[:,:,0].T

potato = 0





#plots the X moments and compares them.
plt.figure(0)
plt.plot(momentsActual[0,:], label='actual')
plt.plot(momentsCalculated[0,:], label='calculated')
plt.legend()
plt.title("X Moments")
plt.show()

#plots the xmoments and compares them.
plt.figure(1)
plt.plot(momentsActual[1,:], label='actual')
plt.plot(momentsCalculated[1,:], label='calculated')
plt.legend()
plt.title("Y Moments")
plt.show()


#plots the moments and compares them.
plt.figure(2)
plt.plot(momentsActual[2,:], label='actual')
plt.plot(momentsCalculated[2,:], label='calculated')
plt.legend()
plt.title("Z Moments")
plt.show()