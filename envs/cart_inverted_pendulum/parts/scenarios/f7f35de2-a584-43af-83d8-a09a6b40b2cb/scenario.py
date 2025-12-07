from csbenchlab.helpers.reference_helpers import *
from csbenchlab.helpers.ic_helpers import *
from csbenchlab.common_types import*
import numpy as np

def scenario(scenario, dt, system_dims):

    overrides = {}
    t_sim = scenario.get("SimulationTime", 20.0)
    reference1 = constant(np.array([0.0, 0.0, 1.0, np.pi]), t_sim/2, dt)
    reference2 = constant(np.array([0.0, 0.0, -5.0, np.pi]), t_sim/2, dt)
    ic = np.zeros((system_dims["Outputs"]))

    # ic = np.array([0.0, 0.0, 3, 0.0])
    ic = np.array([0.0, 0.0, 0.0, np.pi + 0.1])

    return ScenarioOptions(
        reference=np.vstack((reference1, reference2)),
        ic=ic,
        system_parameter_overrides=overrides
    )
