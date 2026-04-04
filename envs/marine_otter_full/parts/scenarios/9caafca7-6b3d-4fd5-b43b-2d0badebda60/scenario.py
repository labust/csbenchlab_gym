from csbenchlab.helpers.reference_helpers import *
from csbenchlab.helpers.ic_helpers import *
from csbenchlab.common_types import*
import numpy as np

def scenario(scenario, dt, system_dims):

    overrides = {}
    lin_steps = generate_steps(scenario, dt, system_dims, [1.5, 0.8,   -1,    2,  -0.8, 0], 0)
    ang_steps = generate_steps(scenario, dt, system_dims, [0,   0.2, -0.2, 0.1, -0.1, 0], 5)
    if lin_steps.shape[0] > ang_steps.shape[0]:
        ang_steps = np.pad(ang_steps, ((0, lin_steps.shape[0] - ang_steps.shape[0]), (0, 0)), mode='edge')
    else:
        lin_steps = np.pad(lin_steps, ((0, ang_steps.shape[0] - lin_steps.shape[0]), (0, 0)), mode='edge')

    reference = lin_steps
    reference[:, 1:] += ang_steps[:, 1:] # skip time column of ang_steps
    ic = np.zeros((system_dims["Outputs"]))

    time = scenario["SimulationTime"]
    overrides = {
        # "payload_mass": Timeseries([35, 35], time=[0, 9*time/20])
        "payload_mass": Timeseries([0, 0], time=[0, 9*time/20])
    }

    return ExperimentOptions(
        reference=reference,
        ic=ic,
        system_parameter_overrides=overrides,
        random_seed=42,
        num_evaluations=1000
    )

