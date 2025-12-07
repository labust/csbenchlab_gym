
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 7421332d-19c9-46fa-8de3-6e7d01457333


@dataclass
class ComponentParams:
    L: int = 25
    A = np.array([0.921,  0,      0.041, 0,\
                 0,      0.918,  0,      0.033,
                 0,      0,      0.924,  0,
                 0,      0,      0,      0.937]).reshape((4, 4))
    B = np.array([0.017,  0.001,
                  0.001,  0.023,
                  0,      0.061,
                  0.072,  0     ]).reshape((4, 2))
    C = np.array([1, 0, 0, 0,
                  0, 1, 0, 0]).reshape((2, 4))
    D = 0.0
    sat_min: float = np.array([-np.inf, -np.inf])
    sat_max: float = np.array([np.inf, np.inf])
    Q = np.eye(2)
    R = np.eye(2) * 0.1
