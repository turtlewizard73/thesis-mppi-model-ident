import os
import yaml
import pandas as pd
import pickle
import numpy as np
import matplotlib.pyplot as plt
from itertools import cycle

import scienceplots
from tabulate import tabulate

from utils.trial_data import TrialData
from utils.util_functions import calculate_avg_std

FIGSIZE = (6, 4)
ms3 = r'$m/s^3$'
rads3 = r'$rad/s^3$'
default_colors = [
    "#1e5167", "#6ab6c7", "#95265c", "#5ab220", "#c86be1",
    "#20f53d", "#e028e5", "#c9dd87", "#4224bf", "#10eddc"]


def eval_trial_data(trial_folder: str) -> TrialData:
    folder = trial_folder
    td = None

    if not os.path.isdir(trial_folder):
        return td

    print(f"___ Processing {folder} ___")
    summary_path = os.path.join(folder, 'summary.yaml')
    with open(summary_path, 'r') as file:
        summary = yaml.safe_load(file)
        print(f"RUN NAME: {summary['name']}")
        print(f"Runs/successful: {summary['runs']}/{summary['successful_trials']}")
        print(f"Whole run time: {summary['loop_time'] / 60:.2f} [mins]")

        td = TrialData(
            name=summary['name'],
            runs=summary['runs'],
            successful_trials=summary['successful_trials'],
            loop_time=summary['loop_time']
        )

    # load metrics and fill TrialData
    df = pd.read_csv(os.path.join(folder, 'final_results.csv'))

    successful_df = df[df['success'] == True]
    for run in successful_df.itertuples():
        metric_path_old = os.path.join(folder, run.metric_path)
        metric_name = os.path.basename(metric_path_old)
        metric_path = os.path.join(folder, 'metrics', metric_name)

        if not os.path.exists(metric_path):
            print(f"Metric not found: {metric_path}")
            continue

        with open(metric_path, 'rb') as file:
            m: ControllerMetric = pickle.load(file)

        td.run_time.append(m.time_elapsed)
        td.frechet_distance.append(m.frechet_distance)
        td.distance_to_goal.append(m.distance_to_goal)
        td.angle_to_goal.append(abs(m.angle_to_goal))
        td.avg_cost.append(m.avg_cost)

        td.avg_linear_velocity.append(m.avg_linear_velocity)
        td.avg_linear_acceleration.append(m.avg_linear_acceleration)
        td.avg_angular_acceleration.append(m.avg_angular_acceleration)
        td.rms_linear_jerk.append(m.rms_linear_jerk)
        td.rms_angular_jerk.append(m.rms_angular_jerk)

    # calculate avarage-s and std-s
    td.rate_of_success = td.successful_trials / td.runs

    td.run_time_avg, td.run_time_std = \
        calculate_avg_std(td.run_time)
    td.frechet_distance_avg, td.frechet_distance_std = \
        calculate_avg_std(td.frechet_distance)
    td.distance_to_goal_avg, td.distance_to_goal_std = \
        calculate_avg_std(td.distance_to_goal)
    td.angle_to_goal_avg, td.angle_to_goal_std = \
        calculate_avg_std(td.angle_to_goal)
    td.avg_cost_avg, td.avg_cost_std = \
        calculate_avg_std(td.avg_cost)

    td.avg_linear_velocity_avg, td.avg_linear_velocity_std = \
        calculate_avg_std(td.avg_linear_velocity)
    td.avg_linear_acceleration_avg, td.avg_linear_acceleration_std = \
        calculate_avg_std(td.avg_linear_acceleration)
    td.avg_angular_acceleration_avg, td.avg_angular_acceleration_std = \
        calculate_avg_std(td.avg_angular_acceleration)
    td.rms_linear_jerk_avg, td.rms_linear_jerk_std = \
        calculate_avg_std(td.rms_linear_jerk)
    td.rms_angular_jerk_avg, td.rms_angular_jerk_std = \
        calculate_avg_std(td.rms_angular_jerk)

    return td


def plot_time_frechet(trial_datas: dict[str, TrialData], colors=default_colors) -> plt.Figure:
    labels = list(trial_datas.keys())

    run_time_avg = [t.run_time_avg for t in trial_datas.values()]
    run_time_std = [t.run_time_std for t in trial_datas.values()]

    x = np.arange(len(labels))
    width = 0.25  # bar width
    fig, ax_time = plt.subplots(num='Run time and Frechet distance', figsize=FIGSIZE)
    bars_time = ax_time.bar(
        x, run_time_avg, width, yerr=run_time_std,
        capsize=5, label='Run time', color=colors[0])
    ax_time.set_ylabel('Run time [s]')
    ax_time.tick_params(axis='y')

    frechet_distance_avg = [t.frechet_distance_avg for t in trial_datas.values()]
    frechet_distance_std = [t.frechet_distance_std for t in trial_datas.values()]

    ax_frechet = ax_time.twinx()
    bars_frechet = ax_frechet.bar(
        x + width, frechet_distance_avg, width, yerr=frechet_distance_std,
        capsize=5, label='Frechet distance', color=colors[1])
    ax_frechet.set_ylabel('Frechet distance [m]')
    ax_frechet.tick_params(axis='y')

    avg_cost_avg = [t.avg_cost_avg for t in trial_datas.values()]
    avg_cost_std = [t.avg_cost_std for t in trial_datas.values()]
    ax_cost = ax_time.twinx()
    ax_cost.spines['right'].set_position(('outward', 60))  # Offset the third axis
    bars_cost = ax_cost.bar(
        x + 2 * width, avg_cost_avg, width, yerr=avg_cost_std,
        capsize=5, label='Average cost', color=colors[2])
    ax_cost.set_ylabel('Average cost [-]')
    ax_cost.tick_params(axis='y')

    # ax_time.set_title('Run Time and Frechet Distance')
    ax_time.set_xticks(x + width)
    ax_time.set_xticklabels(labels)

    axbox = ax_time.get_position()
    fig.legend(
        loc='center',
        bbox_to_anchor=[axbox.x0 + 0.5 * axbox.width, axbox.y0 + axbox.height + 0.05],
        ncol=3)

    # for bars, avg, std in zip(bars_time, run_time_avg, run_time_std):
    #     ax_time.annotate(
    #         f'{avg:.2f} ± {std:.3f} [s]',
    #         xy=(bars.get_x() + bars.get_width() / 2, avg),
    #         xytext=(0, 3),  # 3 points vertical offset
    #         textcoords='offset points',
    #         ha='center', va='bottom')

    # for bars, avg, std in zip(bars_frechet, frechet_distance_avg, frechet_distance_std):
    #     ax_frechet.annotate(
    #         f'{avg:.2f} ± {std:.3f} [m]',
    #         xy=(bars.get_x() + bars.get_width() / 2, avg),
    #         xytext=(0, 3),  # 3 points vertical offset
    #         textcoords='offset points',
    #         ha='center', va='bottom')

    # Print results in a tabular format
    table_data = [
        [
            labels[i],
            f"{run_time_avg[i]:.2f} ± {run_time_std[i]:.3f}",
            f"{frechet_distance_avg[i]:.2f} ± {frechet_distance_std[i]:.3f}",
            f"{avg_cost_avg[i]:.2f} ± {avg_cost_std[i]:.3f}"
        ]
        for i in range(len(labels))
    ]
    table_headers = ["Label", "Run Time [s]", "Frechet Distance [m]", "Average Cost [-]"]
    print(tabulate(table_data, headers=table_headers, tablefmt="grid"))

    return fig


def plot_distance_angle(trial_datas: dict[str, TrialData], colors=default_colors) -> plt.Figure:
    labels = list(trial_datas.keys())
    distance_to_goal_avg = [t.distance_to_goal_avg for t in trial_datas.values()]
    distance_to_goal_std = [t.distance_to_goal_std for t in trial_datas.values()]

    angle_to_goal_avg = [t.angle_to_goal_avg for t in trial_datas.values()]
    angle_to_goal_std = [t.angle_to_goal_std for t in trial_datas.values()]

    x = np.arange(len(labels))
    width = 0.25  # bar width
    fig, ax_distance = plt.subplots(num='Distance and Angle to Goal', figsize=FIGSIZE)
    bars_distance = ax_distance.bar(
        x, distance_to_goal_avg, width, yerr=distance_to_goal_std,
        capsize=5, label='Distance to goal', color=colors[0])
    ax_distance.set_ylabel('Distance to goal [m]')
    ax_distance.tick_params(axis='y')

    ax_angle = ax_distance.twinx()
    bars_angle = ax_angle.bar(
        x + width, angle_to_goal_avg, width, yerr=angle_to_goal_std,
        capsize=5, label='Angle to goal', color=colors[1])
    ax_angle.set_ylabel('Angle to goal [rad]')
    ax_angle.tick_params(axis='y')

    # ax_distance.set_title('Distance and Angle to Goal')
    ax_distance.set_xticks(x + width / 2)
    ax_distance.set_xticklabels(labels)

    axbox = ax_distance.get_position()
    fig.legend(
        loc='center',
        bbox_to_anchor=[axbox.x0 + 0.5 * axbox.width, axbox.y0 + axbox.height + 0.05],
        ncol=2)

    # for bars, avg, std in zip(bars_distance, distance_to_goal_avg, distance_to_goal_std):
    #     ax_distance.annotate(
    #         f'{avg:.2f} ± {std:.3f} [m]',
    #         xy=(bars.get_x() + bars.get_width() / 2, avg),
    #         xytext=(0, 3),  # 3 points vertical offset
    #         textcoords='offset points',
    #         ha='center', va='bottom')

    # for bars, avg, std in zip(bars_angle, angle_to_goal_avg, angle_to_goal_std):
    #     ax_angle.annotate(
    #         f'{avg:.2f} ± {std:.3f} [rad]',
    #         xy=(bars.get_x() + bars.get_width() / 2, avg),
    #         xytext=(0, 3),  # 3 points vertical offset
    #         textcoords='offset points',
    #         ha='center', va='bottom')

    # Print results in a tabular format
    table_date = [
        [
            labels[i],
            f"{distance_to_goal_avg[i]:.2f} ± {distance_to_goal_std[i]:.3f}",
            f"{angle_to_goal_avg[i]:.2f} ± {angle_to_goal_std[i]:.3f}"
        ]
        for i in range(len(labels))
    ]
    table_headers = ["Label", "Distance to Goal [m]", "Angle to Goal [rad]"]
    print(tabulate(table_date, headers=table_headers, tablefmt="grid"))

    return fig


def plot_rms(trial_datas: dict[str, TrialData], colors=default_colors) -> plt.Figure:
    labels = list(trial_datas.keys())
    rms_linear_jerk_avg = [t.rms_linear_jerk_avg for t in trial_datas.values()]
    rms_linear_jerk_std = [t.rms_linear_jerk_std for t in trial_datas.values()]

    x = np.arange(len(labels))
    width = 0.25  # bar width
    fig, ax_linear = plt.subplots(num='RMS Linear and Angular Jerk', figsize=FIGSIZE)
    bars_linear = ax_linear.bar(
        x, rms_linear_jerk_avg, width, yerr=rms_linear_jerk_std,
        capsize=5, label='RMS linear jerk', color=colors[0])
    ax_linear.set_ylabel(f'RMS linear jerk [{ms3}]')
    ax_linear.tick_params(axis='y')

    rms_angular_jerk_avg = [t.rms_angular_jerk_avg for t in trial_datas.values()]
    rms_angular_jerk_std = [t.rms_angular_jerk_std for t in trial_datas.values()]

    ax_angular = ax_linear.twinx()
    bars_angular = ax_angular.bar(
        x + width, rms_angular_jerk_avg, width, yerr=rms_angular_jerk_std,
        capsize=5, label='RMS angular jerk', color=colors[1])
    ax_angular.set_ylabel(f'RMS angular jerk [{rads3}]')
    ax_angular.tick_params(axis='y')

    # ax_linear.set_title('RMS Linear and Angular Jerk')
    ax_linear.set_xticks(x + width / 2)
    ax_linear.set_xticklabels(labels)

    axbox = ax_linear.get_position()
    fig.legend(
        loc='center',
        bbox_to_anchor=[axbox.x0 + 0.5 * axbox.width, axbox.y0 + axbox.height + 0.05],
        ncol=2)

    # for bars, avg, std in zip(bars_linear, rms_linear_jerk_avg, rms_linear_jerk_std):
    #     ax_linear.annotate(
    #         f'{avg:.2f} ± {std:.3f} [m/s^3]',
    #         xy=(bars.get_x() + bars.get_width() / 2, avg),
    #         xytext=(0, 3),  # 3 points vertical offset
    #         textcoords='offset points',
    #         ha='center', va='bottom')

    # for bars, avg, std in zip(bars_angular, rms_angular_jerk_avg, rms_angular_jerk_std):
    #     ax_angular.annotate(
    #         f'{avg:.2f} ± {std:.3f} [rad/s^3]',
    #         xy=(bars.get_x() + bars.get_width() / 2, avg),
    #         xytext=(0, 3),  # 3 points vertical offset
    #         textcoords='offset points',
    #         ha='center', va='bottom')

    # Print results in a tabular format
    table_date = [
        [
            labels[i],
            f"{rms_linear_jerk_avg[i]:.2f} ± {rms_linear_jerk_std[i]:.3f}",
            f"{rms_angular_jerk_avg[i]:.2f} ± {rms_angular_jerk_std[i]:.3f}"
        ]
        for i in range(len(labels))
    ]
    table_headers = ["Label", "RMS Linear Jerk [m/s^3]", "RMS Angular Jerk [rad/s^3]"]
    print(tabulate(table_date, headers=table_headers, tablefmt="grid"))

    return fig


def scatter_time_frechet(trial_datas: dict[str, TrialData], colors=default_colors) -> plt.Figure:
    labels = list(trial_datas.keys())

    fig, ax = plt.subplots(num='Scatter Run Time vs Frechet', figsize=FIGSIZE)
    for i, trial in enumerate(trial_datas.values()):
        x = trial.run_time
        y = trial.frechet_distance
        ax.scatter(x, y, label=labels[i], color=colors[i + 1], alpha=0.5, edgecolors='none')

    ax.set_xlabel('Run Time [s]')
    ax.set_ylabel('Frechet Distance [m]')
    # ax.set_title('Scatter Plot of Run Time vs Frechet Distance')
    ax.legend()

    return fig


def scatter_distance_angle(trial_datas: dict[str, TrialData], colors=default_colors) -> plt.Figure:
    labels = list(trial_datas.keys())

    fig, ax = plt.subplots(num='Scatter Distance to Goal vs Angle to Goal', figsize=FIGSIZE)
    for i, trial in enumerate(trial_datas.values()):
        x = trial.distance_to_goal
        y = trial.angle_to_goal
        ax.scatter(x, y, label=labels[i], color=colors[i + 1], alpha=0.5, edgecolors='none')

    ax.set_xlabel('Distance to Goal [m]')
    ax.set_ylabel('Angle to Goal [rad]')
    # ax.set_title('Scatter Plot of Distance to Goal vs Angle to Goal')
    ax.legend()

    return fig


def scatter_rms(trial_datas: dict[str, TrialData], colors=default_colors) -> plt.Figure:
    labels = list(trial_datas.keys())

    fig, ax = plt.subplots(num='Scatter RMS Linear Jerk vs RMS Angular Jerk', figsize=FIGSIZE)
    for i, trial in enumerate(trial_datas.values()):
        x = trial.rms_linear_jerk
        y = trial.rms_angular_jerk
        ax.scatter(x, y, label=labels[i], color=colors[i + 1], alpha=0.5, edgecolors='none')

    ax.set_xlabel(f'RMS Linear Jerk [{ms3}]')
    ax.set_ylabel(f'RMS Angular Jerk [{rads3}]')
    # ax.set_title('Scatter Plot of RMS Linear Jerk vs RMS Angular Jerk')
    ax.legend()

    return fig


def eval_grid(trial_folder: str, critic_list: list[str], colors=default_colors) -> plt.Figure:
    if not os.path.isdir(trial_folder):
        print(f"Invalid directory: {trial_folder}")
        return None

    # Read the summary.yaml file
    summary_path = os.path.join(trial_folder, 'summary.yaml')
    if not os.path.exists(summary_path):
        print(f"Summary file not found: {summary_path}")
        return None

    with open(summary_path, 'r') as file:
        summary = yaml.safe_load(file)
        print(f"___ Processing {trial_folder} ___")
        print(f"RUN NAME: {summary['name']}")
        print(f"Runs/successful: {summary['runs']}/{summary['successful_trials']}")
        print(f"Whole run time: {summary['loop_time'] / 60:.2f} [mins]")

    # Prepare the plot
    stuff_to_plot = [
        'time_elapsed',
        'score',
        'distance_to_goal',
        'angle_to_goal',
        'avg_cost',
        'rms_linear_jerk',
        'rms_angular_jerk',
    ]

    ylabels = {
        'time_elapsed': 'Time elapsed [s]',
        'score': 'Score [-]',
        'distance_to_goal': 'Distance to goal [m]',
        'angle_to_goal': 'Angle to goal [rad]',
        'avg_cost': 'Average cost [-]',
        'rms_linear_jerk': f'RMS linear jerk [{ms3}]',
        'rms_angular_jerk': f'RMS angular jerk [{rads3}]',
    }

    figures = []
    for metric in stuff_to_plot:
        print(f"Plotting {metric}")

        fig, ax = plt.subplots(num=f'Grid of {metric}', figsize=FIGSIZE)
        color_cycle = cycle(colors)
        labels = []

        for critic in critic_list:
            path = os.path.join(trial_folder, f'{critic}.csv')

            if not os.path.exists(path):
                print(f"File not found: {path}")
                continue

            df = pd.read_csv(path)

            if f'{critic}.cost_weight' not in df or metric not in df:
                print(f"Missing required columns in {path}")
                continue

            ax.plot(
                df[f'{critic}.cost_weight'],
                df[metric],
                label=critic,
                color=next(color_cycle)
            )
            labels.append(critic)

        ax.set_xlabel("Cost Weight")
        ax.set_ylabel(ylabels[metric])
        # ax.set_title("Grid Evaluation")
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        figures.append(fig)
    return figures


def plot_bayes(trial_datas: dict[str, TrialData], colors=default_colors) -> plt.Figure:
    stuff_to_plot = [
        'score',
        'distance_to_goal',
        'angle_to_goal',
        'avg_cost',
        'rms_linear_jerk',
        'rms_angular_jerk',
    ]

    ylabels = {
        'score': 'Score [-]',
        'distance_to_goal': 'Distance to goal [m]',
        'angle_to_goal': 'Angle to goal [rad]',
        'avg_cost': 'Average cost [-]',
        'rms_linear_jerk': f'RMS linear jerk [{ms3}]',
        'rms_angular_jerk': f'RMS angular jerk [{rads3}]',
    }

    figures = []
    for metric in stuff_to_plot:
        fig, ax = plt.subplots(num=f'Bayesian optimization: {metric}', figsize=FIGSIZE)

        for i, (name, data) in enumerate(trial_datas.items()):
            id_ints = np.arange(len(data['id']))
            ax.plot(id_ints, data[metric], label=name, color=colors[i])

        # ax.set_title(metric)
        ax.set_xlabel('iteration')
        ax.set_ylabel(ylabels[metric])
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        figures.append(fig)

    return figures
