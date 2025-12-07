from csbenchlab.helpers.metric_helpers import *
import numpy as np

# Reference file for metric 896e23b8-f0b6-4ecc-a5ad-2edf7b0051eb

# Implement this to generate reference values for the metric
def metric(results):
    out_with_ref(results, ref_dimensions=[2], out_dimensions=[2, 3], linewidth=1.5)
