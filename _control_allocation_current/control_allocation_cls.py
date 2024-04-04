#Control Allocation Function

#this file implements the control allocation function that minimizes energy wasted on hover mode

#imports the system and os
import sys
import os

#gets the path so that we can see outside of this folder
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

#imports numpy
import numpy as np
#imports the scipy optimize library
from scipy.optimize import linprog

#imports the convergence parameters
import parameters.convergence_parameters as VTOL_converge

#imports the 