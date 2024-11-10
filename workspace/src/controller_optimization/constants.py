# paths
REPO_PATH = '/home/turtlewizard/thesis-mppi-model-ident'
BASE_PATH = REPO_PATH + '/workspace/src/controller_optimization'
LAUNCH_PATH = REPO_PATH + '/workspace/src/controller_launch'
OPTIMIZATION_OUTPUT_PATH = REPO_PATH + '/optimization_results'
CONFIG_PATH = REPO_PATH + '/workspace/src/controller_optimization/config'

RESULTS_PATH = REPO_PATH + '/results'
METRICS_PATH = REPO_PATH + '/metrics'

DEFAULT_MPPI_CRITIC_NAMES = [
    'ConstraintCritic', 'GoalCritic', 'GoalAngleCritic', 'PreferForwardCritic',
    'CostCritic', 'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic']

# Occupancy grid values
OCCUPANCY_FREE = 0
OCCUPANCY_UNKNOWN = -1
OCCUPANCY_WALL = 100
