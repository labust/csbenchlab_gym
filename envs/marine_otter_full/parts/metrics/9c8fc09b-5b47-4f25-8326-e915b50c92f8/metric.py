from csbenchlab.helpers.metric_helpers import *
from csbenchlab.common_types import*
import numpy as np

# Reference file for metric 9c8fc09b-5b47-4f25-8326-e915b50c92f8

# Implement this to generate reference values for the metric

@matlab_function('out_with_ref',
    RefDimensions=[1, 6], OutDimensions=[1, 6], LineWidth=1.5, Grid='on',
    SplitPlots=True, FigureSize=[700, 400], Interpreter='none',
    XLabel='Time [s]', YLabel=['Linear Velocity [m/s]', 'Angular Velocity [rad/s]']
)
def metric(results):
    pass
