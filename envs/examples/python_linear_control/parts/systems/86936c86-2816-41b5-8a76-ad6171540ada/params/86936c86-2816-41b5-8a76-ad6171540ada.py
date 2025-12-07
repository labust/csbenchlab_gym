
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 86936c86-2816-41b5-8a76-ad6171540ada


@dataclass
class ComponentParams:
    # give me a good linear system as example
    A = np.array([[0.9, 0.1], [0.0, 0.8]])
    B = np.array([[0.1], [0.2]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])
    sat_min: float = -np.inf  ### EVALUATED FROM DEFAULT FUNCTION IF NONE
    sat_max: float = np.inf