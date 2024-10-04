import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))

import sympy as sp

import parameters.quad.anaconda_parameters as QUAD
from IPython.display import display


#creates the deltas
delta_t_f = sp.symbols('delta_{t_f}')
delta_t_v1 = sp.symbols('delta_{t_{v1}}')
delta_t_v2 = sp.symbols('delta_{t_{v2}}')
delta_t_v3 = sp.symbols('delta_{t_{v3}}')
delta_t_v4 = sp.symbols('delta_{t_{v4}}')

