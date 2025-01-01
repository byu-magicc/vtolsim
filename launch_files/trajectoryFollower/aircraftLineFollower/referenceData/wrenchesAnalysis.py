#This file implements the analyzer to plot the wrenches of the vtol through time


#imports the needed libraries
import os, sys
from pathlib import Path

import numpy as np

import matplotlib.pyplot as plt
import pandas as pd


#reads in the true wrenches csv


#gets the csv file
#gets the absolute path
absPathTrueWrenches = os.path.abspath("launch_files/trajectoryFollower/aircraftLineFollower/referenceData")
tempPathTrueWrenches = absPathTrueWrenches + "/trueWrenches.csv"

actualPath = "/home/benjamin/Documents/vtolsim/launch_files/trajectoryFollower/aircraftLineFollower/referenceData/trueWrenches.csv"

trueWrenchesFile = (pd.read_csv(tempPathTrueWrenches))

tempPathGeometricControllerOutput = absPathTrueWrenches + "/GeometricControllerOutput.csv"

#gets the Geometric controller wrench output
GeoCtrlWrenchFile = pd.read_csv(tempPathGeometricControllerOutput)


#gets the trueWrenches
trueWrenches = trueWrenchesFile.values

#gets the geometric controller wrenches
GeoCtrlWrenches = GeoCtrlWrenchFile.values

#plots each of the forces

for i in range(3):
    plt.figure(0)
    if i == 0:
        mainLabel = 'Fx'
    elif i == 1:
        mainLabel = 'Fy'
    elif i == 2:
        mainLabel = 'Fz'
    plt.plot(trueWrenches[i,:], label='True Wrench')
    plt.plot(GeoCtrlWrenches[i,:], label='Geometric Control Wrench')
    plt.title(mainLabel + " forces")
    plt.legend()
    plt.show()


potato = 0