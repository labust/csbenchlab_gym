
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 0f79c310-23a7-4b0d-9134-99211e5e6d76


@load_from_file("data/m200.mat")
class ComponentParams:
    A: int = 1
    B: int = 1
    C: int = 1
    D: int = 1
    Tini: int = 1
    is_incremental: int = 0
    use_ref_integral: int = 0
    Ki: int = 0
    u_min: Any = None
    u_max: Any = None
    y_min: Any = None
    y_max: Any = None
    base_variable_name: int = 0
