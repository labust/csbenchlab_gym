
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 6582568e-6a07-4ca1-973d-ce726013b9e5


@dataclass
class ComponentParams:
    mu = np.zeros((12,))
    sigma = np.diag([0.02, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
