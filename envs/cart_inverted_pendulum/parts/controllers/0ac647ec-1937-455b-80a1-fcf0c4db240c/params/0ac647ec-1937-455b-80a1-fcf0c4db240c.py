
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 0ac647ec-1937-455b-80a1-fcf0c4db240c


@dataclass
class ComponentParams:
    Kp = np.array([-1.5117,  -47.0233,   -1.2257, -122.7421]).reshape((1,4))
    Ki: float = 0.0
    Kd: float = 0.0
    sat_min: float = -np.inf
    sat_max: float = np.inf