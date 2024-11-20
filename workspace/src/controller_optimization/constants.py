from enum import Enum

# paths
REPO_PATH = '/home/turtlewizard/thesis-mppi-model-ident'
BASE_PATH = REPO_PATH + '/workspace/src/controller_optimization'
LAUNCH_PATH = REPO_PATH + '/workspace/src/controller_launch'
LOG_PATH = REPO_PATH + '/logs'
OPTIMIZATION_OUTPUT_PATH = REPO_PATH + '/optimization_results'
RESULTS_PATH = REPO_PATH + '/results'
METRICS_PATH = REPO_PATH + '/metrics'

BENCHMARK_CONFIG_PATH = BASE_PATH + '/config/controller_benchmark_config.yaml'
OPTIMIZER_CONFIG_PATH = BASE_PATH + '/config/controller_optimizer_config.yaml'

# controller
DEFAULT_MPPI_PARAMS_PATH = LAUNCH_PATH + '/config/nav2_params_benchmark.yaml'
DEFAULT_MPPI_CRITIC_NAMES = [
    'ConstraintCritic', 'GoalCritic', 'GoalAngleCritic', 'PreferForwardCritic',
    'CostCritic', 'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic']

# Occupancy grid values
OCCUPANCY_FREE = 0
OCCUPANCY_UNKNOWN = -1
OCCUPANCY_WALL = 100

# ControllerBenchmark
TIMESTAMP_FORMAT = '%Y-%m-%d_%H-%M-%S'
DEFAULT_BENCHMARK_CONFIG = BASE_PATH + '/config/controller_benchmark_config.yaml'


class Robots(Enum):
    BURGER = 'burger'
    WAFFLE = 'waffle'
    ENJOY = 'enjoy'
