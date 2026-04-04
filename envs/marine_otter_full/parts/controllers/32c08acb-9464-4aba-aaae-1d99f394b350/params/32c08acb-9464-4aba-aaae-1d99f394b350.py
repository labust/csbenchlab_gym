
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 32c08acb-9464-4aba-aaae-1d99f394b350


@load_from_file('data/params.mat')
class ComponentParams:
    solution_path: int = 1
    Tini: int = 1
    is_incremental: int = 0
    use_ref_integral: int = 0
    Ki: int = 0
    u_min: Any = None
    u_max: Any = None
    y_max: Any = None
    y_min: Any = None
    out_gain: int = 1
