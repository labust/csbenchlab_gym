from csbenchlab.helpers.reference_helpers import *
from csbenchlab.helpers.ic_helpers import *
from csbenchlab.common_types import*
import numpy as np

def scenario(scenario, dt, system_dims):

    overrides = {}
    reference = generate_steps(scenario, dt, system_dims, [0.8, 1.5, -1, 2, 0], 0)
    ic = np.zeros((system_dims["Outputs"]))

    return ScenarioOptions(
        reference=reference,
        ic=ic,
        system_parameter_overrides=overrides,
        random_seed=42,
        num_evaluations=1000
    )

