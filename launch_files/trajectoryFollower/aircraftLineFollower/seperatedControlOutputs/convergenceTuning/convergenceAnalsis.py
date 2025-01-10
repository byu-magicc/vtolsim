#This file implements the analysis for the convergence for the controller
import numpy as np
import pandas as pd


import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[4]))

import matplotlib.pyplot as plt

absPath = os.path.abspath("launch_files/trajectoryFollower/aircraftLineFollower/seperatedControlOutputs/convergenceTuning")

deltaOutput = 