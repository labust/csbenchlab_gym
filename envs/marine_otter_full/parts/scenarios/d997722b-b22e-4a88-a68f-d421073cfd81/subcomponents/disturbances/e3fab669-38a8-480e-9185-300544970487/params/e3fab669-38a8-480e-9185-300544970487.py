
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id e3fab669-38a8-480e-9185-300544970487


@dataclass
class ComponentParams:
    mu = np.zeros((12,))
    sigma = np.diag([0.05, 0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
