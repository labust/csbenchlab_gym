
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id d00b25fa-299c-4dde-a026-233da86bc131


@dataclass
class ComponentParams:
    mu = np.zeros((12,))
    sigma = np.diag([0.1, 0.0, 0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
