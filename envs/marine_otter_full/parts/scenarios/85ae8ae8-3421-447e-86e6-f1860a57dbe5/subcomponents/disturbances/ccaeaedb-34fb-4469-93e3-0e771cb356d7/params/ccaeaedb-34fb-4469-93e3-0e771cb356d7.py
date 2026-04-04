
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id ccaeaedb-34fb-4469-93e3-0e771cb356d7


@dataclass
class ComponentParams:
    mu = np.zeros((12,))
    sigma = np.diag([0.05, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
