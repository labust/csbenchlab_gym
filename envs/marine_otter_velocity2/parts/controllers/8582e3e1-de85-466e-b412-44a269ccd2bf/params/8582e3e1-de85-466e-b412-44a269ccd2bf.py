
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import LoadFromFile, PyFunctionHandle, MatFunctionHandle
import numpy as np

# Parameter file for component with id 8582e3e1-de85-466e-b412-44a269ccd2bf


@dataclass
class ComponentParams:
    K_p = 57.0360395290896,
    K_i = 45.0805551814842,
    K_d = 0,
    K_N = 100,
    saturation_max = 500
    saturation_min = -300
