
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 941af73e-12e7-42cf-853d-f79bb40068e7


@dataclass
class ComponentParams:
    mu = np.zeros((12,))
    sigma = np.diag([0.02, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
