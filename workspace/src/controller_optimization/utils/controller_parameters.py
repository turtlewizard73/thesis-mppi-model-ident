from dataclasses import dataclass, field
from typing import List, Any
import yaml
import numpy as np


DEFAULT_MPPI_CRITIC_NAMES = [
    'ConstraintCritic', 'GoalCritic', 'GoalAngleCritic', 'PreferForwardCritic',
    'CostCritic', 'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic']


@dataclass
class ControllerCritic:
    """ControllerCritic dataclass."""
    # some critics have more options those are tuned by hand
    name: str
    cost_weight: float = 1.0
    cost_power: float = 1.0


@dataclass
class MPPIControllerParameters:
    """MPPIController parameters dataclass."""
    name: str = ''
    critic_names: List[str] = field(
        default_factory=lambda: DEFAULT_MPPI_CRITIC_NAMES)

    critics: List[ControllerCritic] = field(
        default_factory=lambda: [ControllerCritic(n) for n in DEFAULT_MPPI_CRITIC_NAMES])

    def __str__(self):
        # print pretty the critics
        critics_str = f'[MPPIControllerParameters] {self.name} critics:'
        for critic in self.critics:
            critics_str += f'\n{critic.name}: {critic.cost_weight}, {critic.cost_power}'

        return critics_str

    def _find_mppi_controller(self, data: dict) -> Any:
        """Recursively searches for the MPPIController configuration."""
        search_key = 'nav2_mppi_controller::MPPIController'
        if isinstance(data, dict):
            for key, value in data.items():
                if key == self.name:
                    return value

                if isinstance(value, dict) and value.get('plugin') == search_key:
                    self.name = key
                    return value  # Return the whole dict for the MPPIController
                # Recursively search in nested dictionaries
                result = self._find_mppi_controller(value)
                if result:
                    return result

        return None  # If not found

    def load_from_yaml(self, file_path: str):
        """Loads the MPPIController parameters from a YAML file."""
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
        """Converts the MPPIController parameters to a dictionary."""
        if self.name == '':
            raise ValueError('MPPIController name is not set.')

        mppi_dict = {}
        for critic in self.critics:
            mppi_dict.update({
                f'{self.name}.{critic.name}.cost_weight': critic.cost_weight,
                f'{self.name}.{critic.name}.cost_power': critic.cost_power
            })

        return mppi_dict

    def randomize_weights(
            self, distribution: str = 'uniform',
            lower_bound: float = 0.01, upper_bound: float = 100.0,
            avg=None, std_dev=None) -> None:
        """Randomizes the MPPIController parameters."""
        for critic in self.critics:
            if distribution == 'uniform':
                critic.cost_weight = np.random.uniform(lower_bound, upper_bound)
            elif distribution == 'normal' and avg is not None and std_dev is not None:
                critic.cost_weight = np.clip(np.random.normal(avg, std_dev), lower_bound, upper_bound)
