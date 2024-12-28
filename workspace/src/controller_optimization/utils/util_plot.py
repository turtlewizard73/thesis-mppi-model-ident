from utils.util_functions import calculate_avg_std
from utils.trial_data import TrialData
import os
import yaml
import pandas as pd
import pickle
import numpy as np
import matplotlib.pyplot as plt
import scienceplots

colors = ['blue', 'salmon', 'green', 'purple', 'orange']


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
            m = pickle.load(file)

        td.run_time.append(m.time_elapsed)
        td.frechet_distance.append(m.frechet_distance)
        td.distance_to_goal.append(m.distance_to_goal)
        td.angle_to_goal.append(m.angle_to_goal)
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


def plot_time_frechet(trial_datas: dict[str, TrialData]) -> plt.Figure:
    labels = list(trial_datas.keys())
    run_time_avg = [t.run_time_avg for t in trial_datas.values()]
    run_time_std = [t.run_time_std for t in trial_datas.values()]

    frechet_distance_avg = [t.frechet_distance_avg for t in trial_datas.values()]
    frechet_distance_std = [t.frechet_distance_std for t in trial_datas.values()]

    x = np.arange(len(labels))
    width = 0.25  # bar width
    fig, ax_time = plt.subplots(num='Run time and Frechet distance')
    bars_time = ax_time.bar(
        x, run_time_avg, width, yerr=run_time_std, capsize=5, label='Run Time')
    ax_time.set_ylabel('Run Time [s]')
    ax_time.tick_params(axis='y')

    ax_frechet = ax_time.twinx()
    bars_frechet = ax_frechet.bar(
        x + width, frechet_distance_avg, width, yerr=frechet_distance_std,
        capsize=5, label='Frechet Distance',
        color='salmon')
    ax_frechet.set_ylabel('Run Time [s]')
    ax_frechet.tick_params(axis='y')

    ax_time.set_title('Run Time and Frechet Distance')
    ax_time.set_xticks(x + width / 2)
    ax_time.set_xticklabels(labels)
    fig.legend(loc='lower left', ncol=2)

    for bars, avg, std in zip(bars_time, run_time_avg, run_time_std):
        ax_time.annotate(
            f'{avg:.2f} \u00B1 {std:.3f} [s]',
            xy=(bars.get_x() + bars.get_width() / 2, avg),
            xytext=(0, 3),  # 3 points vertical offset
            textcoords='offset points',
            ha='center', va='bottom')

    for bars, avg, std in zip(bars_frechet, frechet_distance_avg, frechet_distance_std):
        ax_frechet.annotate(
            f'{avg:.2f} \u00B1 {std:.3f} [m]',
            xy=(bars.get_x() + bars.get_width() / 2, avg),
            xytext=(0, 3),  # 3 points vertical offset
            textcoords='offset points',
            ha='center', va='bottom')

    return fig


def plot_distance_angle(trial_datas: dict[str, TrialData]) -> plt.Figure:
    labels = list(trial_datas.keys())
    distance_to_goal_avg = [t.distance_to_goal_avg for t in trial_datas.values()]
    distance_to_goal_std = [t.distance_to_goal_std for t in trial_datas.values()]

    angle_to_goal_avg = [t.angle_to_goal_avg for t in trial_datas.values()]
    angle_to_goal_std = [t.angle_to_goal_std for t in trial_datas.values()]

    x = np.arange(len(labels))
    width = 0.25  # bar width
    fig, ax_distance = plt.subplots(num='Distance and Angle to Goal')
    bars_distance = ax_distance.bar(
        x, distance_to_goal_avg, width, yerr=distance_to_goal_std,
        capsize=5, label='Distance to Goal')
    ax_distance.set_ylabel('Distance to Goal [m]')
    ax_distance.tick_params(axis='y')

    ax_angle = ax_distance.twinx()
    bars_angle = ax_angle.bar(
        x + width, angle_to_goal_avg, width, yerr=angle_to_goal_std,
        capsize=5, label='Angle to Goal',
        color='salmon')
    ax_angle.set_ylabel('Angle to Goal [rad]')
    ax_angle.tick_params(axis='y')

    ax_distance.set_title('Distance and Angle to Goal')
    ax_distance.set_xticks(x + width / 2)
    ax_distance.set_xticklabels(labels)
    fig.legend(loc='lower left', ncol=2)

    for bars, avg, std in zip(bars_distance, distance_to_goal_avg, distance_to_goal_std):
        ax_distance.annotate(
            f'{avg:.2f} \u00B1 {std:.3f} [m]',
            xy=(bars.get_x() + bars.get_width() / 2, avg),
            xytext=(0, 3),  # 3 points vertical offset
            textcoords='offset points',
            ha='center', va='bottom')

    for bars, avg, std in zip(bars_angle, angle_to_goal_avg, angle_to_goal_std):
        ax_angle.annotate(
            f'{avg:.2f} \u00B1 {std:.3f} [rad]',
            xy=(bars.get_x() + bars.get_width() / 2, avg),
            xytext=(0, 3),  # 3 points vertical offset
            textcoords='offset points',
            ha='center', va='bottom')

    return fig


def plot_rms(trial_datas: dict[str, TrialData]) -> plt.Figure:
    labels = list(trial_datas.keys())
    rms_linear_jerk_avg = [t.rms_linear_jerk_avg for t in trial_datas.values()]
    rms_linear_jerk_std = [t.rms_linear_jerk_std for t in trial_datas.values()]

    rms_angular_jerk_avg = [t.rms_angular_jerk_avg for t in trial_datas.values()]
    rms_angular_jerk_std = [t.rms_angular_jerk_std for t in trial_datas.values()]

    x = np.arange(len(labels))
    width = 0.25  # bar width
    fig, ax_linear = plt.subplots(num='RMS Linear and Angular Jerk')
    bars_linear = ax_linear.bar(
        x, rms_linear_jerk_avg, width, yerr=rms_linear_jerk_std,
        capsize=5, label='RMS Linear Jerk')
    ax_linear.set_ylabel('RMS Linear Jerk [m/s^3]')
    ax_linear.tick_params(axis='y')

    ax_angular = ax_linear.twinx()
    bars_angular = ax_angular.bar(
        x + width, rms_angular_jerk_avg, width, yerr=rms_angular_jerk_std,
        capsize=5, label='RMS Angular Jerk',
        color='salmon')
    ax_angular.set_ylabel('RMS Angular Jerk [rad/s^3]')
    ax_angular.tick_params(axis='y')

    ax_linear.set_title('RMS Linear and Angular Jerk')
    ax_linear.set_xticks(x + width / 2)
    ax_linear.set_xticklabels(labels)
    fig.legend(loc='lower left', ncol=2)

    for bars, avg, std in zip(bars_linear, rms_linear_jerk_avg, rms_linear_jerk_std):
        ax_linear.annotate(
            f'{avg:.2f} \u00B1 {std:.3f} [m/s^3]',
            xy=(bars.get_x() + bars.get_width() / 2, avg),
            xytext=(0, 3),  # 3 points vertical offset
            textcoords='offset points',
            ha='center', va='bottom')

    for bars, avg, std in zip(bars_angular, rms_angular_jerk_avg, rms_angular_jerk_std):
        ax_angular.annotate(
            f'{avg:.2f} \u00B1 {std:.3f} [rad/s^3]',
            xy=(bars.get_x() + bars.get_width() / 2, avg),
            xytext=(0, 3),  # 3 points vertical offset
            textcoords='offset points',
            ha='center', va='bottom')

    return fig


def plot_avg_cost(trial_datas: dict[str, TrialData]) -> plt.Figure:
    labels = list(trial_datas.keys())
    avg_cost_avg = [t.avg_cost_avg for t in trial_datas.values()]
    avg_cost_std = [t.avg_cost_std for t in trial_datas.values()]

    x = np.arange(len(labels))
    width = 0.25  # bar width
    fig, ax_cost = plt.subplots(num='Average Cost')
    bars_cost = ax_cost.bar(
        x, avg_cost_avg, width, yerr=avg_cost_std,
        capsize=5, label='Average Cost')
    ax_cost.set_ylabel('Average Cost [-]')
    ax_cost.tick_params(axis='y')

    ax_cost.set_title('Average Cost')
    ax_cost.set_xticks(x)
    ax_cost.set_xticklabels(labels)
    fig.legend(loc='lower left', ncol=2)

    return fig


def scatter_time_frechet(trial_datas: dict[str, TrialData]) -> plt.Figure:
    labels = list(trial_datas.keys())

    fig, ax = plt.subplots()
    for i, trial in enumerate(trial_datas.values()):
        x = trial.run_time
        y = trial.frechet_distance
        ax.scatter(x, y, label=labels[i], color=colors[i], alpha=0.3, edgecolors='none')

    ax.set_xlabel('Run Time [s]')
    ax.set_ylabel('Frechet Distance [m]')
    ax.set_title('Scatter Plot of Run Time vs Frechet Distance')
    ax.legend()

    return fig


def scatter_distance_angle(trial_datas: dict[str, TrialData]) -> plt.Figure:
    labels = list(trial_datas.keys())

    fig, ax = plt.subplots()
    for i, trial in enumerate(trial_datas.values()):
        x = trial.distance_to_goal
        y = trial.angle_to_goal
        ax.scatter(x, y, label=labels[i], color=colors[i], alpha=0.3, edgecolors='none')

    ax.set_xlabel('Distance to Goal [m]')
    ax.set_ylabel('Angle to Goal [rad]')
    ax.set_title('Scatter Plot of Distance to Goal vs Angle to Goal')
    ax.legend()

    return fig


def scatter_rms(trial_datas: dict[str, TrialData]) -> plt.Figure:
    labels = list(trial_datas.keys())

    fig, ax = plt.subplots()
    for i, trial in enumerate(trial_datas.values()):
        x = trial.rms_linear_jerk
        y = trial.rms_angular_jerk
        ax.scatter(x, y, label=labels[i], color=colors[i], alpha=0.3, edgecolors='none')

    ax.set_xlabel('RMS Linear Jerk [m/s^3]')
    ax.set_ylabel('RMS Angular Jerk [rad/s^3]')
    ax.set_title('Scatter Plot of RMS Linear Jerk vs RMS Angular Jerk')
    ax.legend()

    return fig
