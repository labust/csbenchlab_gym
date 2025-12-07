from csbenchlab.helpers.reference_helpers import *
from csbenchlab.helpers.ic_helpers import *
from csbenchlab.common_types import *
import numpy as np

def scenario(scenario, dt, system_dims):

    overrides = {}
    reference = constant(np.array([0.0, 0.0, 1.0, np.pi]), scenario, dt)
    ic = np.zeros((system_dims["Outputs"]))

    # ic = np.array([0.0, 0.0, 3, 0.0])
    ic = np.array([0.0, 0.0, 0.0, np.pi + 0.1])

    return ScenarioOptions(
        reference=reference,
        ic=ic,
        system_parameter_overrides=overrides
    )
