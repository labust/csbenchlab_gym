from csbenchlab.helpers.reference_helpers import *
from csbenchlab.helpers.ic_helpers import *
from csbenchlab.common_types import*
import numpy as np

def scenario(scenario, dt, system_dims):

    overrides = {}
    lin_f = 0.08
    ang_f = 0.05
    lin_sinuses = generate_sin_reference(scenario, dt, system_dims, [0.5, 1, 1.5, 0], [lin_f] * 4, dim=0)
    ang_sinuses = generate_sin_reference(scenario, dt, system_dims, [0, 0.1, 0.2, 0.1, 0.2], [ang_f] * 5, dim=5)

    if lin_sinuses.shape[0] > ang_sinuses.shape[0]:
        ang_sinuses = np.pad(ang_sinuses, ((0, lin_sinuses.shape[0] - ang_sinuses.shape[0]), (0, 0)), mode='edge')
    else:
        lin_sinuses = np.pad(lin_sinuses, ((0, ang_sinuses.shape[0] - lin_sinuses.shape[0]), (0, 0)), mode='edge')

    reference = lin_sinuses
    reference[:, 1:] += ang_sinuses[:, 1:] # skip time column of ang_sinuses
    ic = np.zeros((system_dims["Outputs"]))

    time = scenario["SimulationTime"]
    overrides = {
        "payload_mass": Timeseries([35, 35], time=[0, 4*time/20])
    }

    return ExperimentOptions(
        reference=reference,
        ic=ic,
        system_parameter_overrides=overrides,
        random_seed=42,
        num_evaluations=1000
    )

