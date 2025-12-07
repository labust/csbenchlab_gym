
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 43a8b674-a981-4b52-9e74-b37728839260


@dataclass
class ComponentParams:
    C = np.array([[2.3689,   52.7379,    1.7160,  135.8368]])  # Sliding surface matrix
    K = 0.0
    F = 1.0
    epsilon = 1
    sat_min = -np.inf
    sat_max = np.inf
