#%%
import sympy as sp
from IPython.display import Latex, display
import numpy as np


x, y = sp.symbols('x, y')

g = sp.Matrix([[x, x**2, y**4],
               [x**3, x**4, y**7]])

s = (x, y)

g_func = sp.lambdify(s, g, modules='numpy')
print(g_func(3,4))
# %%
