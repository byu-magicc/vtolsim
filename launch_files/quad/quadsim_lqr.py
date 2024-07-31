#creates launch file for a vtolsim

import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
import numpy as np
import parameters.simulation_parameters as SIM
from models.quad.quad_dynamics import QuadDynamics

