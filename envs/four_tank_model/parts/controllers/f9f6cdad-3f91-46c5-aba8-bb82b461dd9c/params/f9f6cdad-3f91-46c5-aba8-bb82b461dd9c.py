
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id f9f6cdad-3f91-46c5-aba8-bb82b461dd9c


@dataclass
class ComponentParams:
    Kp = np.array([1, 1, 1, 1, 1, 1, 1, 1]).reshape((2, 4))
    Ki = np.array([0, 0, 0, 0, 0, 0, 0, 0]).reshape((2, 4))
    Kd = np.array([0, 0, 0, 0, 0, 0, 0, 0]).reshape((2, 4))
    sat_min: float = np.array([-np.inf, -np.inf])
    sat_max: float = np.array([np.inf, np.inf])
