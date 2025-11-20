
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 9c6c94ce-2741-455f-a8da-be6b073916ae


@dataclass
class ComponentParams:
    solution_path: int = "data/pwl.mat"
    Tini: int = 3
    is_incremental: int = 0
    use_ref_integral: int = 0
    Ki: int = 0.7
    u_min: Any = -500
    u_max: Any = 500
    y_max: Any = 4
    y_min: Any = -4
