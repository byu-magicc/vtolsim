#this file analyzes the positional erros for the whole flight of the aircraft,
# and we can begin to anaylze how to make it better
import numpy as np
import matplotlib.pyplot as plt

#loads in the pitch free info file
pitchFreeFile = np.load("/home/benjamin/Documents/vtolsim/launch_files/trajectoryFollower/aircraftLineFollower/outputs/PitchFreeInfo.npz")



pos_errs = pitchFreeFile['arr_0']
vel_errs = pitchFreeFile['arr_1']
F_ds = pitchFreeFile['arr_2']
R = pitchFreeFile['arr_3']

#gets  the positional errors shape
pos_errs_shape = np.shape(pos_errs)
pos_errs = np.reshape(pos_errs, (pos_errs_shape[0], pos_errs_shape[1]))


#extracts the x, y, and z errors for the position error
x_error = pos_errs[:,0]
y_error = pos_errs[:,1]
z_error = pos_errs[:,2]

plt.figure(0)
plt.plot(x_error, label = 'x error')
plt.plot(y_error, label = 'y error')
plt.plot(z_error, label = 'z error')
plt.title("Errors")
plt.legend()
plt.show()


pineapple = 0