
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id b11ccfb6-c361-4845-af31-0c9bdb83000b


@load_from_file("data/m.mat")
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
