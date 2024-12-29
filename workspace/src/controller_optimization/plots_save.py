import os
import matplotlib.pyplot as plt
import scienceplots

from utils.util_plot import (
    eval_trial_data,
    plot_time_frechet,
    plot_distance_angle,
    plot_rms,
    scatter_time_frechet, scatter_distance_angle, scatter_rms)

# __________ CONFIG __________
WORK_DIR = '/home/turtlewizard/repos/thesis-mppi-model-ident/optimization_results'

DOCS_PATH = '/home/turtlewizard/repos/thesis-mppi-model-ident/msc-docs'
SAVE_PATH = os.path.join(DOCS_PATH, 'plots')
if not os.path.exists(SAVE_PATH):
    os.makedirs(SAVE_PATH)

# first 4 ok
colors = [
    "#1e5167", "#6ab6c7", "#95265c", "#5ab220", "#c86be1",
    "#20f53d", "#e028e5", "#c9dd87", "#4224bf", "#10eddc"]

DPI = 300

# https://github.com/garrettj403/SciencePlots/blob/master/examples/plot-examples.py
CONTEXT_SCIENCE = plt.style.context(['science'])
CONTEXT_SCIENCE_GRID = plt.style.context(['science', 'grid'])
CONTEXT_SCATTER = plt.style.context(['science', 'scatter'])

ctxs = [CONTEXT_SCIENCE, CONTEXT_SCIENCE_GRID, CONTEXT_SCATTER]

# __________ DISTRIBUTION TRIALS __________
waffle_std_path = WORK_DIR + '/default_distribution_2024-11-24_13-23-28'
burger_std_path = WORK_DIR + '/burger_distribution_2024-11-24_14-35-54'
enjoy_std_path = WORK_DIR + '/enjoy_distribution_2024-11-24_14-35-12'
enjoy2_std_path = WORK_DIR + '/enjoy_distribution_w_enjoy_params_2024-12-27_21-09-52'

waffle_std_data = eval_trial_data(waffle_std_path)
burger_std_data = eval_trial_data(burger_std_path)
enjoy_std_data = eval_trial_data(enjoy_std_path)
enjoy2_std_data = eval_trial_data(enjoy2_std_path)

trial_datas_std = {
    'waffle': waffle_std_data,
    'burger': burger_std_data,
    'enjoy': enjoy_std_data,
    'enjoy2': enjoy2_std_data,
}
# KÉSZ

# __________ RANDOM TRIALS __________
waffle_rand100_path = WORK_DIR + '/waffle_random_2024-12-28_12-42-49'
waffle_rand500_path = WORK_DIR + '/waffle_random_2024-12-28_15-49-04'

waffle_rand100_data = eval_trial_data(waffle_rand100_path)
waffle_rand500_data = eval_trial_data(waffle_rand500_path)

trial_datas_rand = {
    'waffle_random_500': waffle_rand500_data,
    'waffle_random_100': waffle_rand100_data,
    'waffle_default': waffle_std_data,
}

# TODO: process after 500 ran
enjoy_rand100_path = WORK_DIR + ''  # FUT
enjoy_rand500_path = WORK_DIR + ''  # FUT
# ...

# __________ GRID TRIALS __________
waffle_grid_path = WORK_DIR + '/waffle_grid_2024-12-28_12-42-57'
waffle_grid_data = eval_trial_data(waffle_grid_path)
# KÉSZ

# __________ BAYESIAN __________
waffle_bayesian100_path = WORK_DIR + '/waffle_bayesian_2024-12-28_15-47-40'
waffle_bayesian500_path = WORK_DIR + ''  # FUT
enjoy_bayesian100_path = WORK_DIR + ''
enjoy_bayesian500_path = WORK_DIR + ''  # FUT


# __________ PLOT DEFAULT DISTRIBUTION  __________
with CONTEXT_SCIENCE:
    fig_std_time = plot_time_frechet(trial_datas_std, colors)
    fig_std_time.savefig(
        os.path.join(SAVE_PATH, '00_std_time_frechet_cost.pdf'),
        dpi=DPI)

    fig_std_dist = plot_distance_angle(trial_datas_std, colors)
    fig_std_dist.savefig(
        os.path.join(SAVE_PATH, '01_std_distance_angle_cost.pdf'),
        dpi=DPI)

    fig_std_rms = plot_rms(trial_datas_std, colors)
    fig_std_rms.savefig(
        os.path.join(SAVE_PATH, '02_std_rms_cost.pdf'),
        dpi=DPI)

# __________ PLOT RANDOM DISTRIBUTION WAFFLE  __________
with CONTEXT_SCIENCE:
    # the same as above
    fig_rand_time = plot_time_frechet(trial_datas_rand, colors)
    fig_rand_time.savefig(
        os.path.join(SAVE_PATH, '10_rand_time_frechet_cost.pdf'),
        dpi=DPI)

    fig_rand_dist = plot_distance_angle(trial_datas_rand, colors)
    fig_rand_dist.savefig(
        os.path.join(SAVE_PATH, '11_rand_distance_angle_cost.pdf'),
        dpi=DPI)

    fig_rand_rms = plot_rms(trial_datas_rand, colors)
    fig_rand_rms.savefig(
        os.path.join(SAVE_PATH, '12_rand_rms_cost.pdf'),
        dpi=DPI)

    # scatter plots
    fig_rand_scatter_time = scatter_time_frechet(trial_datas_rand, colors)
    fig_rand_scatter_time.savefig(
        os.path.join(SAVE_PATH, '13_rand_scatter_time_frechet_cost.pdf'),
        dpi=DPI)

    fig_rand_scatter_dist = scatter_distance_angle(trial_datas_rand, colors)
    fig_rand_scatter_dist.savefig(
        os.path.join(SAVE_PATH, '14_rand_scatter_distance_angle_cost.pdf'),
        dpi=DPI)

    fig_rand_scatter_rms = scatter_rms(trial_datas_rand, colors)
    fig_rand_scatter_rms.savefig(
        os.path.join(SAVE_PATH, '15_rand_scatter_rms_cost.pdf'),
        dpi=DPI)

    # TODO: get recheck with BEST PARAMS

# __________ PLOT GRID DISTRIBUTION  __________
