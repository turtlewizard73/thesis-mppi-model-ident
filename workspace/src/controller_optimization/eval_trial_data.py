import os
import yaml
import pandas as pd
import pickle
import matplotlib.pyplot as plt
import numpy as np

from utils.trial_data import TrialData
from utils.util_functions import calculate_avg_std


# load summary.yaml
WORK_DIR = '/home/turtlewizard/thesis-mppi-model-ident/evaluation/01_distribution_100'
# WORK_DIR = '/home/turtlewizard/thesis-mppi-model-ident/evaluation/02_random'

trial_datas = {}
for folder in os.listdir(WORK_DIR):
    if not os.path.isdir(os.path.join(WORK_DIR, folder)):
        continue

    print(f"___ Processing {folder} ___")
    td = None
    summary_path = os.path.join(WORK_DIR, folder, 'summary.yaml')
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
    df = pd.read_csv(os.path.join(WORK_DIR, folder, 'final_results.csv'))

    successful_df = df[df['success'] == True]
    for run in successful_df.itertuples():
        metric_path_old = os.path.join(WORK_DIR, folder, run.metric_path)
        metric_name = os.path.basename(metric_path_old)
        metric_path = os.path.join(WORK_DIR, folder, 'metrics', metric_name)

        if not os.path.exists(metric_path):
            print(f"Metric not found: {metric_path}")
            continue

        with open(metric_path, 'rb') as file:
            m = pickle.load(file)

        td.run_time.append(m.time_elapsed)
        td.frechet_distance.append(m.frechet_distance)
        td.distance_to_goal.append(m.distance_to_goal)
        td.angle_to_goal.append(m.angle_to_goal)
        td.sum_of_costs.append(m.sum_of_costs)

        td.avg_linear_velocity.append(m.avg_linear_velocity)
        td.avg_linear_acceleration.append(m.avg_linear_acceleration)
        td.avg_angular_acceleration.append(m.avg_angular_acceleration)
        td.rms_linear_jerk.append(m.rms_linear_jerk)
        td.rms_angular_jerk.append(m.rms_angular_jerk)

    # calculate avarage-s and std-s
    td.rate_of_success = td.successful_trials / td.runs
    n = len(td.run_time)

    td.run_time_avg, td.run_time_std = \
        calculate_avg_std(td.run_time)
    td.frechet_distance_avg, td.frechet_distance_std = \
        calculate_avg_std(td.frechet_distance)
    td.distance_to_goal_avg, td.distance_to_goal_std = \
        calculate_avg_std(td.distance_to_goal)
    td.angle_to_goal_avg, td.angle_to_goal_std = \
        calculate_avg_std(td.angle_to_goal)
    td.sum_of_costs_avg, td.sum_of_costs_std = \
        calculate_avg_std(td.sum_of_costs)

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

    trial_datas.update({str(folder): td})

# print trial data
for folder, data in trial_datas.items():
    print(data.print_trial_data())
    # save data to pickle
    with open(os.path.join(WORK_DIR, f'{folder}_data.pkl'), 'wb') as file:
        pickle.dump(data, file)
