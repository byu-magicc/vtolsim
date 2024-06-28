"""
    Simulate using equilibrium throttle commands to find a
    linear motor model.
"""

import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

from tools.rotations import quaternion_to_rotation

#imports the Quad dynamics
from models.quad.quad_dynamics import QuadDynamics
from tools.trim import *
import parameters.quad.anaconda_parameters as QUAD
import parameters.simulation_parameters as SIM
import numpy as np

np.set_printoptions(precision=4, linewidth=200, suppress=True)


