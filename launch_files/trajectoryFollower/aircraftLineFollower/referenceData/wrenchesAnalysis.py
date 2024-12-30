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
absPath = os.path.abspath("launch_files/trajectoryFollower/aircraftLineFollower/referenceData")
tempPath = absPath + "/trueWrenches.csv"

actualPath = "/home/benjamin/Documents/vtolsim/launch_files/trajectoryFollower/aircraftLineFollower/referenceData/trueWrenches.csv"

trueWrenchesFile = (pd.read_csv(absPath + "/trueWrenches.csv"))

#gets the trueWrenches
trueWrenches = trueWrenchesFile.values


#plots each of the forces

plt.figure(0)
plt.plot(trueWrenches[0,:], label="Fx")
plt.plot(trueWrenches[1,:], label="Fy")
plt.plot(trueWrenches[2,:], label="Fz")
plt.legend()
plt.title("True Steady State Forces comparison")
plt.show()

potato = 0