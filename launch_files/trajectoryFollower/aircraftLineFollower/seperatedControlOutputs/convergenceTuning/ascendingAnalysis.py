#this file implements the convergence analysis for the ascending aircraft

import numpy as np


import numpy as np
import pandas as pd


import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[4]))

import matplotlib.pyplot as plt

#sets the cutoff for good data
cutoff = 300

absPath = os.path.abspath("launch_files/trajectoryFollower/aircraftLineFollower/seperatedControlOutputs/convergenceTuning")


#section to get the output for 104
#gets the delta output file
deltaOutput_104 = pd.read_csv(absPath + '/deltaOutputArray_104.csv', index_col=False, header=None).values
#gets the state output
stateOutput_104 = pd.read_csv(absPath + '/stateOutputArray_104.csv', index_col=False, header=None).values

trajectoryOutput_104 = pd.read_csv(absPath + '/trajectoryOutputArray_104.csv', index_col=False, header=None).values

omegaDesired_104 = pd.read_csv(absPath + '/OmegaDesired_104.csv', index_col=False, header=None).values

ForcesDesired_104 = pd.read_csv(absPath + '/ForceDesired_104.csv', index_col=False, header=None).values

ForcesMomentsActual_104 = pd.read_csv(absPath + '/ForcesMomentsActual_104.csv', index_col=False, header=None).values

ForcesDesiredInertial_104 = pd.read_csv(absPath + '/ForcesDesiredInertial_104.csv', index_col=False, header=None).values




#'''
#plots the z inertial forces, as well as the altitude
plt.figure(0)
plt.plot(-stateOutput_104[2,:cutoff], label='altitude')
plt.plot(-trajectoryOutput_104[2,:cutoff], label='trajectory')
plt.legend()
plt.title("Z Force and Altitude Comparison")
plt.show()

plt.figure(1)
plt.plot(ForcesDesiredInertial_104[2,:cutoff],label='Fz Desired Inertial')
plt.legend()
plt.title("Z Forces")
plt.show()
#'''
