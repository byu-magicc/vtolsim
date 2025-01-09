#This file does the work of comparing different control inputs and the 

import numpy as np
import pandas as pd


import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[4]))

import matplotlib.pyplot as plt

#-----------------------------------Delta Comparisons---------------------------------
#in this section, we are reading in the deltas from the output files, and then plotting 
#them out such that we can figure out if there is any improvement. 



#gets the absolute path to the main folder
path = os.path.abspath("launch_files/trajectoryFollower/aircraftLineFollower/seperatedControlOutputs/geometricControllerTuning/deltaOutputs")

#reads from the csv file in the delta output
deltaOutputs_1 = pd.read_csv(path + '/deltaOutputs_1.csv', index_col=False, header=None).values

#reads in the csv file for the delta output 2
deltaOutputs_2 = pd.read_csv(path + '/deltaOutputs_2.csv', index_col=False, header=None).values

#plots the four currently used control outputs

#elevator
plt.figure(0)
plt.plot(deltaOutputs_1[0,:], label='outputs 1')
plt.plot(deltaOutputs_2[0,:], label='outputs 2')
plt.legend()
plt.title("Elevator")
plt.show()


#aileron
plt.figure(1)
plt.plot(deltaOutputs_1[1,:], label='outputs 1')
plt.plot(deltaOutputs_2[1,:], label='outputs 2')
plt.legend()
plt.title("Aileron")
plt.show()

#elevator
plt.figure(2)
plt.plot(deltaOutputs_1[2,:], label='outputs 1')
plt.plot(deltaOutputs_2[2,:], label='outputs 2')
plt.legend()
plt.title("Rudder")
plt.show()

#elevator
plt.figure(3)
plt.plot(deltaOutputs_1[3,:], label='outputs 1')
plt.plot(deltaOutputs_2[3,:], label='outputs 2')
plt.legend()
plt.title("Forward Throttle")
plt.show()


potato = 0



