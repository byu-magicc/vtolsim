import numpy as np

VecStart_x = [0,1,3,5]
VecStart_y = [2,2,5,5]
VecStart_z = [0,1,1,5]
VecEnd_x = [1,2,-1,6]
VecEnd_y = [3,1,-2,7]
VecEnd_z  =[1,0,4,9]

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x = [0,1,2,3]
y = [0,1,2,3]
z = [0,0,0,0]

ax.plot(x,y,z)
plt.show()
Axes3D.plot()

pinearpple = 0
