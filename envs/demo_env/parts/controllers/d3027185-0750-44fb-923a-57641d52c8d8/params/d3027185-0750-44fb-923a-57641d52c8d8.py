
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id d3027185-0750-44fb-923a-57641d52c8d8


@dataclass
class ComponentParams:
    p1: int = 0
    p2: np.ndarray = np.array([1, 2, 3])
    p12: PyFunctionHandle = None  ### EVALUATED FROM DEFAULT FUNCTION IF NONE
