
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id a2d09404-1425-4551-b056-e0738041c105


@dataclass
class ComponentParams:
    K = Any = None
    sat_min: float = -np.inf
    sat_max: float = np.inf
    synthesize: bool = True
    A = np.array([[-0.2000,       0,         0,    2.0000],
                 [0,   -0.1000,         0,    6.0000],
                 [1.0000,         0,         0,         0],
                 [0,    1.0000,         0,         0]
    ])
    B = np.array([[0.2000],
                  [-0.1000],
                  [0],
                  [0]])
    C = np.eye(4)
    Q = np.diag([1.0, 1.0, 10.0, 10.0])
    R = 1e-3