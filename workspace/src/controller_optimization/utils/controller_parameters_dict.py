from dataclasses import dataclass, field
from typing import List, Any
import yaml
import numpy as np
from typing import TypedDict


DEFAULT_MPPI_CRITIC_NAMES = [
    'ConstraintCritic', 'GoalCritic', 'GoalAngleCritic', 'PreferForwardCritic',
    'CostCritic', 'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic']


@dataclass
class ControllerCriticDict(TypedDict):
    """ControllerCritic dataclass."""
    # some critics have more options those are tuned by hand
    name: str
    cost_weight: float
    cost_power: float


@dataclass
class ControllerParametersDict(TypedDict):
   controller_name: str
   critics: List[ControllerCriticDict] = field(
