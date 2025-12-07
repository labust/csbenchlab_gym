
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 1e9fb6a3-f53f-480c-a2d0-a3b9797384f4


@dataclass
class ComponentParams:
    A = np.array([[0,  -1/5,   981/500, 0],
        [0, -1/60, -981/1000, 0],
        [1,     0,         0, 0],
        [0,     1,         0, 0]])
    B: np.ndarray = np.array([[1/5],
        [12/5],
        [0],
        [0]])
    C = np.eye(4)
    D: np.ndarray = np.array([[0.0]])
    sat_min: float = -np.inf
    sat_max: float = np.inf
