from csbenchlab.helpers.reference_helpers import *
import numpy as np

# Reference file for scenario 65576e1b-c335-442d-a66f-c0a831447138

# Implement this to generate reference values for the scenario
def reference(scenario, dt, system_dims):
    return generate_steps(scenario, dt, system_dims, [1, 0, 1, 0], 0)

