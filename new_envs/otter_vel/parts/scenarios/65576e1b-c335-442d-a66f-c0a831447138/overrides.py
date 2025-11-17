# Override system parameters file for scenario 65576e1b-c335-442d-a66f-c0a831447138

from csbenchlab.param_typing import MultiScenario
from csbenchlab.helpers.reference_helpers import *

# Implement this to override certain system parameters for the scenario
def override_system_params(system_params):
    return MultiScenario(system_params, 1000, generator=gen)


def gen(params):
    import numpy as np
    params["param1"] = np.random.uniform(0, 10)
    params["param2"] = np.random.randint(1, 5)
    return params