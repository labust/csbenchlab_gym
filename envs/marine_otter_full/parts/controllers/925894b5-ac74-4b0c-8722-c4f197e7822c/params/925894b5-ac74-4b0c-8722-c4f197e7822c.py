
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 925894b5-ac74-4b0c-8722-c4f197e7822c


@load_from_file('data/params.mat')
class ComponentParams:
    A: int = 1
    B: int = 1
    C: int = 1
    D: int = 1
    Tini: int = 1
    solution_path: int = 0
    is_incremental: int = 0
    use_ref_integral: int = 0
    Ki: int = 0
    u_min: Any = None
    u_max: Any = None
    y_min: Any = None
    y_max: Any = None
    base_variable_name: int = 0
    out_gain: int = 1
