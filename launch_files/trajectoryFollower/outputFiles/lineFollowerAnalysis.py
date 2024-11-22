#reads in the npz files
import numpy as np
import matplotlib.pyplot as plt


stateArrayFile = np.load('/home/dben1182/Documents/vtolsim/launch_files/trajectoryFollower/outputFiles/state.npz')
stateTimeArray = stateArrayFile['arr_0']

#gets the number of samples
numSamples = len(stateTimeArray)

#gets the stateVectorLength
stateVectorLen = len(stateTimeArray[0])



for i in range(stateVectorLen):
    
    tempSignal = np.ndarray((0,1))
    for j in range(numSamples):
        currentState = stateTimeArray[j]
        tempSignal = np.concatenate((tempSignal, np.array([[(currentState).item(i)]])), axis=0)
    
    plt.figure(i)
    plt.plot(tempSignal)
    plt.title(i)
    plt.show()
    pineapple = 0




pine = 0