
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.param_typing import *
import numpy as np

# Parameter file for component with id c3911dad-90a4-4212-8c73-b2279d7035c0


@dataclass
class ComponentParams:
    A: int = 1
    B: int = 1
    C: int = 1
    D: MatFunctionHandle = None  ### EVALUATED FROM DEFAULT FUNCTION IF NONE
    sat_min: MatFunctionHandle = None  ### EVALUATED FROM DEFAULT FUNCTION IF NONE
    sat_max: MatFunctionHandle = None  ### EVALUATED FROM DEFAULT FUNCTION IF NONE
