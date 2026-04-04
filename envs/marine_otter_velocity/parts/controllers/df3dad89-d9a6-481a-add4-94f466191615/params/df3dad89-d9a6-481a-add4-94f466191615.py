
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

# Parameter file for component with id df3dad89-d9a6-481a-add4-94f466191615


@dataclass
class ComponentParams:
    L: int = 25
    Tini: int = 3
    Ts: int = 0.05
    D_u = [-31.021,-31.021,-31.021,-31.021,-31.021,-31.021,-31.021,-31.021,-31.021,-31.021,
           -31.021,-31.021,-31.021,-31.021,-31.021,-31.021,-31.021,-31.021,-31.021,30.4362,
           30.4362,30.4362,30.4362,30.4362,30.4362,30.4362,30.4362,30.4362,30.4362,30.4362,
           30.4362,30.4362,30.4362,30.4362,30.4362,30.4362,30.4362,30.4362,30.4362,41.3329,
           41.3329,41.3329,41.3329,41.3329,41.3329,41.3329,41.3329,41.3329,41.3329,41.3329,
           41.3329,41.3329,41.3329,41.3329,41.3329,41.3329,41.3329,41.3329,41.3329,32.6139,
           32.6139,32.6139,32.6139,32.6139,32.6139,32.6139,32.6139,32.6139,32.6139,32.6139,
           32.6139,32.6139,32.6139,32.6139,32.6139,32.6139,32.6139,32.6139,32.6139,42.3376]
    D_y = [0.35558,0.35675,0.35537,0.35173,0.34609,0.3387,0.32978,0.31952,0.30811,0.29572,
           0.28248,0.26854,0.25403,0.23904,0.22369,0.20806,0.19224,0.17629,0.16028,0.14427,
           0.12794,0.11428,0.10304,0.093975,0.086879,0.081551,0.077813,0.075497,0.074453,
           0.074542,0.075636,0.077621,0.08039,0.083847,0.087905,0.092482,0.097506,0.10291,
           0.10864,0.11463,0.12078,0.12756,0.13488,0.14265,0.1508,0.15926,0.16797,0.17687,
           0.18592,0.19507,0.20427,0.2135,0.22272,0.23189,0.24101,0.25004,0.25897,0.26777,
           0.27644,0.28496,0.29338,0.30126,0.30864,0.31556,0.32203,0.3281,0.33379,0.33912,
           0.34412,0.3488,0.35319,0.3573,0.36116,0.36478,0.36817,0.37135,0.37433,0.37713,0.37975,0.38222]
    end_point: float = 0.0129
    R: float = 1.0
    Q: float = 1.0
    lambda_g: float = 1e4
    lambda_s: float = 1e6
    lambda_s_ini: float = 1e6
    lambda_term_u: float = 1e5
    lambda_term_y: float = 1e5
    terminal_constraint_size: int = 1
    affine_constraint: int = 1
    use_overshoot_constraints: int = 0
    use_input_terminal_constraints: int = 1
    use_input_delta_constraints: int = 0
    use_projected_regularization: int = 0
    is_strict_terminal_constraint: int = 0
    use_ref_integral: int = 0
    Ki: int = 1
    decay: int = 1
    allowed_offset_terminal: int = 1
    allowed_offset_slack: int = 1
    u_min: float = -300.0
    u_max: float = 500.0
    y_min: float = -3
    y_max: float = 3
    input_delta: float = 100.0
    out_gain = 1
    use_ini_filter = 0
    T: PyFunctionHandle = None  ### EVALUATED FROM DEFAULT FUNCTION IF NONE
    Lc: PyFunctionHandle = None  ### EVALUATED FROM DEFAULT FUNCTION IF NONE
