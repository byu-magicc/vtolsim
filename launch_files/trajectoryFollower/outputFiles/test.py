import numpy as np



counter = 0
End = 10

array = []

while counter < End:

    temp = np.array([[counter], [counter], [counter]])

    array.append(temp)
    counter += 1

