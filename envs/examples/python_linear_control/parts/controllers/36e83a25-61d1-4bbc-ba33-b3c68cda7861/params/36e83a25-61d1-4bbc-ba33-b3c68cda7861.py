
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 36e83a25-61d1-4bbc-ba33-b3c68cda7861


@dataclass
class ComponentParams:
    L: int = 25
    A = np.array([[0.9, 0.1], [0.0, 0.8]])
    B = np.array([[0.1], [0.2]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])
    Q = 1
    R = 0.001
    sat_min: float = -5
    sat_max: float = 5