#This file implements the analysis for the convergence for the controller
import numpy as np
import pandas as pd


import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[4]))

import matplotlib.pyplot as plt

#sets the cutoff for good data
cutoff = 250

absPath = os.path.abspath("launch_files/trajectoryFollower/aircraftLineFollower/seperatedControlOutputs/convergenceTuning")


#section to get the output for 103
#gets the delta output file
deltaOutput_103 = pd.read_csv(absPath + '/deltaOutputArray_103.csv', index_col=False, header=None).values
#gets the state output
stateOutput_103 = pd.read_csv(absPath + '/stateOutputArray_103.csv', index_col=False, header=None).values

trajectoryOutput_103 = pd.read_csv(absPath + '/trajectoryOutputArray_103.csv', index_col=False, header=None).values

omegaDesired_103 = pd.read_csv(absPath + '/OmegaDesired_103.csv', index_col=False, header=None).values

ForcesDesired_103 = pd.read_csv(absPath + '/ForceDesired_103.csv', index_col=False, header=None).values

ForcesMomentsActual_103 = pd.read_csv(absPath + '/ForcesMomentsActual_103.csv', index_col=False, header=None).values

ForcesDesiredInertial_103 = pd.read_csv(absPath + '/ForceDesired_103.csv', index_col=False, header=None).values

#section to get the output for 104
#gets the delta output file
deltaOutput_104 = pd.read_csv(absPath + '/deltaOutputArray_104.csv', index_col=False, header=None).values
#gets the state output
stateOutput_104 = pd.read_csv(absPath + '/stateOutputArray_104.csv', index_col=False, header=None).values

trajectoryOutput_104 = pd.read_csv(absPath + '/trajectoryOutputArray_104.csv', index_col=False, header=None).values

omegaDesired_104 = pd.read_csv(absPath + '/OmegaDesired_104.csv', index_col=False, header=None).values

ForcesDesired_104 = pd.read_csv(absPath + '/ForceDesired_104.csv', index_col=False, header=None).values

ForcesMomentsActual_104 = pd.read_csv(absPath + '/ForcesMomentsActual_104.csv', index_col=False, header=None).values



north_inertial_103 = stateOutput_103[0,:]
north_ref_inertial_103 = trajectoryOutput_103[0,:]
north_inertial_104 = stateOutput_104[0,:]
north_ref_inertial_104 = trajectoryOutput_104[0,:]


east_inertial_103 = stateOutput_103[1,:]
east_ref_inertial_103 = trajectoryOutput_103[1,:]
east_inertial_104 = stateOutput_104[1,:]
east_ref_inertial_104 = trajectoryOutput_104[1,:]

altitude_inertial_103 = -stateOutput_103[2,:]
altitude_ref_inertial_103 = -trajectoryOutput_103[2,:]
altitude_inertial_104 = -stateOutput_104[2,:]
altitude_ref_inertial_104 = -trajectoryOutput_104[2,:]


#plots all the above information out
plt.figure(2)
plt.plot(altitude_inertial_103[:cutoff], label='altitude 103')
plt.plot(altitude_inertial_104[:cutoff], label='altitude 104')
plt.plot(altitude_ref_inertial_103[:cutoff], label='altitude ref 103')
plt.plot(altitude_ref_inertial_104[:cutoff], label='altitude ref 104')
plt.legend()
plt.title("Altitude Plot")
plt.show()



#goes through and gets the 




#plots a comparison of the desired to the actual forces





potato = 0