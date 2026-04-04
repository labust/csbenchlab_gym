from csbenchlab.helpers.reference_helpers import *
from csbenchlab.helpers.ic_helpers import *
from csbenchlab.common_types import*
import numpy as np

def scenario(scenario, dt, system_dims):

    time = scenario["SimulationTime"]
    overrides = {
        "payload_mass": Timeseries([0, 35], time=[0, 9*time/20])
    }
    reference = generate_steps(scenario, dt, system_dims, [1, 0, 1, 0], 0)
    ic = np.zeros((system_dims["Outputs"]))

    return ScenarioOptions(
        reference=reference,
        ic=ic,
        system_parameter_overrides=overrides
    )

