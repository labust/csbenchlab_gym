
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id 59d3b695-31bd-4d38-a8d0-9c2b61ca3ebd


@dataclass
class ComponentParams:
    L: int = 12
    Tini: int = 5
    Ts: int = 0.05
    R = [1, 5]
    Q = [0.5, 5]
    lambda_g = 2e2
    lambda_s = [1e3, 1e3]
    lambda_s_ini = [1e3, 1e3]
    lambda_term_u = [1e3, 1e3]
    lambda_term_y = [1e3, 1e3]
    terminal_constraint_size = 1
    affine_constraint = 1
    use_overshoot_constraints: int = 0
    use_input_terminal_constraints: int = 1
    use_input_delta_constraints: int = 1
    use_projected_regularization: int = 1
    is_strict_terminal_constraint: int = 0
    use_ref_integral: int = 1
    Ki = [0.8, 0.5]
    trend_threshold = [0.02, 0.02]
    decay: int = 0.8
    allowed_offset_terminal: int = 1
    allowed_offset_slack: int = 1
    u_min = [-1.0, -0.8]
    u_max = [1.0, 0.8]
    y_min = [-3, -0.3]
    y_max = [3, 0.3]
    input_delta = [0.2, 0.2]
    out_gain = 1
    use_ini_filter = 1
    params_path = CSPath("data/ds_bank.mat")
    T: PyFunctionHandle = None  ### EVALUATED FROM DEFAULT FUNCTION IF NONE
    Lc: PyFunctionHandle = None  ### EVALUATED FROM DEFAULT FUNCTION IF NONE
