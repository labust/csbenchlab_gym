from csbenchlab.helpers.metric_helpers import *
import numpy as np

# Reference file for metric a8ebe5c8-5899-478b-ba2d-41718a58b0a6

# Implement this to generate reference values for the metric
def metric(results):
    out_with_ref(results, out_dimensions=[0], ref_dimensions=[0],
                 grid=True, xlabel="Time (s)", ylabel="Velocity (m/s)",)
