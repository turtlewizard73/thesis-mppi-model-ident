# paths
BASE_PATH = '/home/turtlewizard/thesis-mppi-model-ident/workspace/src/controller_optimization'
LAUNCH_PATH = '/home/turtlewizard/thesis-mppi-model-ident/workspace/src/controller_launch'

DEFAULT_MPPI_CRITIC_NAMES = [
    'ConstraintCritic', 'GoalCritic', 'GoalAngleCritic', 'PreferForwardCritic',
    'CostCritic', 'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic']

# Occupancy grid values
OCCUPANCY_FREE = 0
OCCUPANCY_UNKNOWN = -1
OCCUPANCY_WALL = 100
