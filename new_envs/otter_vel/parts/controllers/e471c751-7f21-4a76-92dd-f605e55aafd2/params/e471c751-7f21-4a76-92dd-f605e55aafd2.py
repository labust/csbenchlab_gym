
from dataclasses import dataclass, field
from typing import List, Dict, Any
from csbenchlab.common_types import *
import numpy as np

@dataclass
@load_from_file('params.mat', 'Params')
class ComponentParams:
    pass
