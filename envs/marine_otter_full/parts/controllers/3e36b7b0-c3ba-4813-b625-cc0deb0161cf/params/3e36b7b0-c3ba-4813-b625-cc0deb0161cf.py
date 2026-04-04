
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 3e36b7b0-c3ba-4813-b625-cc0deb0161cf


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
