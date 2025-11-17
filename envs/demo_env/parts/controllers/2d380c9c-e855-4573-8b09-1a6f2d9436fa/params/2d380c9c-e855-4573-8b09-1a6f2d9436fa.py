
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 2d380c9c-e855-4573-8b09-1a6f2d9436fa


@dataclass
class ComponentParams:
    p1: int = 2
    p2: int = 3
    p12: MatFunctionHandle = None  ### EVALUATED FROM DEFAULT FUNCTION IF NONE
