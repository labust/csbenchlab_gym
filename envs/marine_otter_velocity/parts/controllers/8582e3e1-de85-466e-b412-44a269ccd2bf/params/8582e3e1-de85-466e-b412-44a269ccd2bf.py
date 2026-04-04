
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import LoadFromFile, PyFunctionHandle, MatFunctionHandle
import numpy as np

# Parameter file for component with id 8582e3e1-de85-466e-b412-44a269ccd2bf


@dataclass
class ComponentParams:
    K_p = np.array([57.0360395290896]) * 1 / 200
    K_i = np.array([45.0805551814842]) * 1 / 300
    K_d = np.array([0.0])
    K_N = np.array([0.2])
    saturation_max = np.array([1])
    saturation_min = np.array([-1])
