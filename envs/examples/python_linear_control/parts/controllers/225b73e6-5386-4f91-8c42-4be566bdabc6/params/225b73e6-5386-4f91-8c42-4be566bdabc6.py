
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 225b73e6-5386-4f91-8c42-4be566bdabc6


@dataclass
class ComponentParams:
    Kp: float = 1.0
    Ki: float = 0.0
    Kd: float = 0.0
    sat_min: float = -np.inf
    sat_max: float = np.inf
