from csbenchlab.helpers.ic_helpers import *
import numpy as np

# Initial conditions file for scenario 65576e1b-c335-442d-a66f-c0a831447138

# Implement this to generate initial conditions for the scenario
def ic(scenario, system_dims):
    return np.zeros(system_dims["Outputs"])
