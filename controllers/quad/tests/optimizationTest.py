#%%

#this file is to attempt methods to do minimization


import scipy as sp
import numpy as np

def function(xy):
    x, y = xy
    return np.sin(x)*np.sin(y)



sp.optimize.minimize(function, (0.1, -0.1))
# %%


import scipy as sp
import numpy as np

from scipy.optimize import minimize

#defines the objective function
def objective(x):
    x0 = x[0]
    x1 = x[1]
    x2 = x[2]
    x3 = x[3]
    #returns the value of the function
    return x0*x3*(x0+x1+x2)+x2


#defines the constraint number 1
def constraint1(x):
    return x[0]*x[1]*x[2]*x[3] - 25.0

#defines the constraint number 2
def constraint2(x):
    sum_sq = 40.0
    for i in range(4):
        sum_sq = sum_sq - x[i]**2
    
    return sum_sq

x0 = [1, 5, 5, 1]
print(objective(x0))


b = (1.0, 5.0)
bounds = (b, b, b, b)
con1 = {'type': 'ineq', 'fun': constraint1}
con2 = {'type': 'ineq', 'fun': constraint2}

cons = [con1, con2]

minimize(objective, x0=x0, method='SLSQP', bounds=bounds, constraints=cons)





# %%
