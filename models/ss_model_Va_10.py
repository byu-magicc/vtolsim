import numpy as np
Va = 10.000000
gamma = 0.000000
trim_state = np.array([[0.890764, 0.000005, -0.000008, 9.592141, 0.000000, 2.826805, 0.989751, 0.000000, 0.142804, 0.000000, 0.000000, 0.000000, 0.000000, 0.311178, ]]).T
trim_input = np.array([[0.051252, 0.456583, 0.000000, 0.995468, 0.985707, 0.059923, 0.311178, 0.261016, ]]).T
A = np.array([
[0.000000, 0.000000, 0.000000, 0.959214, 0.000000, 0.282681, -0.003995, -0.050002, -0.050000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, -2.826758, 0.000000, 9.999833, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, -0.282681, 0.000000, 0.959214, -0.013557, -9.999833, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 1.891855, -0.000558, -10.755265, 0.000000, -9.395868, 0.000000, 0.000000, -2.578182, 0.000000, -0.881172, -0.672997],
[0.000000, 0.000000, 0.000000, 0.000771, -0.522056, 0.000228, 9.409733, 0.000000, 0.000000, 2.917875, 0.000000, -9.255882, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, -2.262181, -0.000698, 2.805062, -0.047049, -2.820100, 0.000000, 0.000000, 8.748496, 0.000000, -4.346699, -4.281679],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.294700, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000031, 0.000000, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -0.000000, 0.000000, 1.042520, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 2.260556, -4.500625, 0.633786, 0.000000, 0.000000, 0.000000, -20.844001, 0.000000, 3.605647, -51.169675, 53.125867],
[0.000000, 0.000000, 0.000000, -1.414956, -0.001206, -4.216931, 0.000000, 0.000000, 0.000000, -0.000019, -3.919971, 0.000019, 20.864156, 20.552060],
[0.000000, 0.000000, 0.000000, -0.249023, 9.266507, -0.129315, 0.000000, 0.000000, 0.000000, -3.156740, 0.000000, -6.118601, 8.708606, -2.129826],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -10.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -10.000000],
])
B = np.array([[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
[0.849408, 0.000000, 0.000000, 7.097599, 7.155514, 0.000000, 0.000000, 0.000000],
[0.000000, 0.008799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
[-3.172658, 0.000000, 0.000000, -2.282779, -1.911310, -0.701909, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
[0.000000, 25.466399, 0.000000, -34.574494, 16.460982, 0.000077, 0.000000, 0.000000],
[-10.851537, 0.000000, 0.000000, 10.957339, 9.174288, -6.738329, 0.000000, 0.000000],
[0.000000, -2.672694, 0.000000, -49.124006, 51.848046, 0.026418, 0.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 10.000000, 0.000000],
[0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 10.000000],
])