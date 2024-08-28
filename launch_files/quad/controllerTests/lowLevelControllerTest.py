#this file's purpose is to perform a test of the low level controller 

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))

import numpy as np
import parameters.quad.simulation_parameters as SIM

from message_types.msgConvert 