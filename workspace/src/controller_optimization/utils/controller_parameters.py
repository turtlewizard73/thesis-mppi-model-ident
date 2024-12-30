from dataclasses import dataclass, field
from typing import List, Any
import yaml
import numpy as np
# from typing import TypedDict


DEFAULT_MPPI_CRITIC_NAMES = [
    'ConstraintCritic', 'GoalCritic', 'GoalAngleCritic', 'PreferForwardCritic',
    'CostCritic', 'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic']


# class ControllerCriticDict(TypedDict):
#     """ControllerCritic dataclass."""
#     # some critics have more options those are tuned by hand
#     name: str
#     cost_weight: float
#     cost_power: float


# class ControllerParametersDict(TypedDict):
#     controller_name: str
#     critics: List[ControllerCriticDict]


@dataclass
class ControllerCritic:
    """ControllerCritic dataclass."""
    # some critics have more options those are tuned by hand
    name: str
    cost_weight: float = 1.0
    cost_power: float = 1.0


@dataclass
class ControllerParameters:
    """Controller parameters dataclass."""
    controller_name: str = 'FollowPath'
    critics: List[ControllerCritic] = field(
        default_factory=lambda: [ControllerCritic(n) for n in DEFAULT_MPPI_CRITIC_NAMES])

    def __str__(self):
        # print pretty the critics
        critics_str = f'[ControllerParameters] {self.controller_name} critics:'
        for critic in self.critics:
            critics_str += f'\n{critic.name}: {critic.cost_weight}, {critic.cost_power}'

        return critics_str

    def set_critic_weight(self, critic_name: str, weight: float):
        for critic in self.critics:
            if critic.name == critic_name:
                critic.cost_weight = float(weight)
                return
        raise ValueError(f'Critic {critic_name} not found in ControllerParameters.')

    def get_critic_weight(self, critic_name: str) -> float:
        for critic in self.critics:
            if critic.name == critic_name:
                return critic.cost_weight
        raise ValueError(f'Critic {critic_name} not found in ControllerParameters.')

    def _find_mppi_controller(self, data: dict) -> Any:
        """Recursively searches for the Controller configuration."""
        if isinstance(data, dict):
            for key, value in data.items():
                if key == self.controller_name:
                    return value

                # Recursively search in nested dictionaries
                result = self._find_mppi_controller(value)
                if result:
                    return result

        return None  # If not found

    def load_from_yaml(self, file_path: str):
        """Loads the Controller parameters from a YAML file."""

        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

            mppi_plugin_dict = self._find_mppi_controller(data)

            for critic in self.critics:
                critic_dict = mppi_plugin_dict.get(critic.name)
                if critic_dict:
                    critic.cost_weight = critic_dict.get('cost_weight', 1.0)
                    critic.cost_power = critic_dict.get('cost_power', 1.0)
                else:
                    raise ValueError(f'No critic configuration found for {critic.name}.')

    def to_dict(self) -> dict:
        """Converts the Controller parameters to a dictionary."""
        mppi_dict = {}
        for critic in self.critics:
            mppi_dict.update({
                f'{critic.name}.cost_weight': critic.cost_weight,
                f'{critic.name}.cost_power': critic.cost_power
            })

        return mppi_dict

    def randomize_weights(
            self, distribution: str = 'uniform',
            lower_bound: float = 0.01, upper_bound: float = 100.0,
            decimals: int = 1,
            avg=None, std_dev=None) -> None:
        """Randomizes the Controller parameters."""
        for critic in self.critics:
            if distribution == 'uniform':
                cost_weight = np.random.uniform(lower_bound, upper_bound)
            elif distribution == 'normal' and avg is not None and std_dev is not None:
                cost_weight = np.clip(np.random.normal(
                    avg, std_dev), lower_bound, upper_bound)
            else:
                raise ValueError('Unsupported distribution or missing parameters.')

            critic.cost_weight = float(np.round(cost_weight, decimals))
