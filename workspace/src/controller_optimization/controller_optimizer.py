import os
import logging
from pprint import pformat
import yaml
import time
from typing import List, Dict, TypedDict, Callable
from enum import Enum
from copy import deepcopy
import numpy as np
import pandas as pd
import traceback
import pickle

# bayesian stuff
from skopt import Optimizer
from skopt import gp_minimize
from skopt.space import Real
from skopt.utils import use_named_args

import constants
from utils.util_functions import timing_decorator
from utils.controller_parameters import ControllerParameters
from utils.controller_metrics import ControllerMetric
from controller_benchmark import ControllerBenchmark

BASE_PATH = constants.BASE_PATH


class GeneratorType(Enum):
    DEFAULT = 'default'
    RANDOM = 'random'
    GRID = 'grid'
    BEST = 'best'
    BAYESIAN = 'bayesian'
    BAYESIAN_BEST = 'bayesian_best'
    BAYESIAN_FEED = 'bayesian_feed'
    BEST_BEST = 'best_best'


class Trial(TypedDict):
    name: str
    map: str
    robot: str
    generator: str
    score_method: str
    runs: int
    run_timeout: float


class RunResult(TypedDict):
    id: str
    success: bool
    time_elapsed: float
    score: float = 0.0
    frechet_distance: float = 0.0
    distance_to_goal: float = 0.0
    angle_to_goal: float = 0.0
    avg_cost: float = 0.0
    rms_linear_jerk: float = 0.0
    rms_angular_jerk: float = 0.0
    status_msg: str
    metric_path: str = 'None'


class ControllerOptimizer:
    def __init__(self, logger, config_path: str):
        self.logger = logger
        self.config_path = config_path
        self.params: Dict = {}
        self.configured_trials: List[Trial] = []
        self.work_dir = ''
        self.load_config()

        self.cb = ControllerBenchmark(
            logger=logger.getChild('Benchmark'),
            config_path=constants.BENCHMARK_CONFIG_PATH)
        self.cb.launch_nodes()

        self.final_output_rows: List[Dict] = []

        # trial parameters
        self.current_trial: Trial = {}
        self.generator: Callable = None

        # reference parameters
        # TODO: put these into config then into load_config()
        self.reference_score = 0.0
        self.reference_params = ControllerParameters()
        self.reference_params.load_from_yaml(constants.DEFAULT_MPPI_PARAMS_PATH)
        self.reference_metric = None
        self.reference_metric_path = 'None'

        # test parameters
        self.test_params = deepcopy(self.reference_params)

    @timing_decorator(
        lambda self: self.logger.info('Loading params...'),
        lambda self, ex_time: self.logger.info(f'Params loaded in {ex_time} s.'))
    def load_config(self):
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config file not found: {self.config_path}")

        with open(self.config_path, 'r') as file:
            data = yaml.safe_load(file)
            self.params['trials'] = data['trials']

            for t in self.params['trials']:
                t = Trial(
                    name=t,
                    map=data[t]['map'],
                    robot=data[t]['robot'],
                    generator=data[t]['generator'],
                    score_method=data[t]['score_method'],
                    runs=data[t]['runs'],
                    run_timeout=data[t]['run_timeout']
                )
                self.configured_trials.append(t)

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug(f'Params: \n {pformat(self.params)}')
            self.logger.debug(f'Config: \n {pformat(self.configured_trials)}')

    def setup_work_dir(self):
        stamp = time.strftime(constants.TIMESTAMP_FORMAT)
        folder_name = f"{self.current_trial['name']}_{stamp}"
        self.work_dir = os.path.join(
            constants.OPTIMIZATION_OUTPUT_PATH, folder_name)
        if not os.path.exists(self.work_dir):
            os.makedirs(self.work_dir)

        self.output_csv_path = os.path.join(self.work_dir, 'run_results.csv')
        self.output_final_csv_path = os.path.join(self.work_dir, 'final_results.csv')
        self.output_yaml_path = os.path.join(self.work_dir, 'summary.yaml')

        # overwrite paths
        self.cb.result_save_path = os.path.join(self.work_dir, 'results')
        self.cb.metric_save_path = os.path.join(self.work_dir, 'metrics')

    def setup_run(
            self, trial_name: str, timeout: float = 0.0,
            generator: Callable = None, score_method: Callable = None) -> None:
        self.logger.info(f"Setting up run for trial: {trial_name}")
        # set current trial
        trial = next(t for t in self.configured_trials if t['name'] == trial_name)

        # overwrite timout
        if timeout > 0.0:
            self.current_trial['run_timeout'] = timeout

        # overwrite benchmark configs
        self.cb.params['robot_name'] = trial['robot']

        # set the generator function
        if generator is None:
            match GeneratorType(trial['generator']):
                case GeneratorType.DEFAULT:
                    self.generator = self.generator_default
                case GeneratorType.RANDOM:
                    self.generator = self.generator_random
                case GeneratorType.GRID:
                    self.generator = self.generator_grid
                case GeneratorType.BEST:
                    self.generator = self.generator_default_best
                case GeneratorType.BAYESIAN:
                    self.generator = self.generator_bayesian
                case GeneratorType.BAYESIAN_BEST:
                    self.generator = self.generator_bayesian_best
                case GeneratorType.BAYESIAN_FEED:
                    self.generator = self.generator_bayesian_feed
                case GeneratorType.BEST_BEST:
                    self.generator = self.generator_best_best
                case _:
                    raise ValueError(f"Invalid generator type: {trial['generator']}")
        else:
            trial['generator'] = generator.__name__
            self.generator = generator

        # set score function
        self.current_score = 0.0
        if score_method is None:
            self.logger.info(f'Using score method: {trial["score_method"]}')
            match trial['score_method']:
                case 'default':
                    self.score_method = self.score_default
                case 'frechet':
                    self.score_method = self.score_frechet
                case 'time':
                    self.score_method = self.score_time
                case 'jerk':
                    self.score_method = self.score_jerk
                case 'avarage':
                    self.score_method = self.score_avarage
                case _:
                    raise ValueError(f"Invalid score method: {trial['score_method']}")
        else:
            trial['score_method'] = score_method.__name__
            self.score_method = score_method

        self.current_trial = trial

        # setup work directory
        self.setup_work_dir()

        self.logger.info(f"Setup complete for trial: {trial_name}")

    def generator_default(self):
        n = self.current_trial['runs']
        self.test_params = deepcopy(self.reference_params)
        for i in range(1, n + 1):
            # do not change parameters
            yield i

    def generator_default_best(self):
        n = self.current_trial['runs']
        best_metric, best_params = self.get_best_metric()
        score = self.score_method(best_metric)

        self.test_params = deepcopy(best_params)
        self.cb.update_parameters(self.test_params)

        for i in range(1, n + 1):
            # do not change parameters
            yield i

    def generator_best_best(self):
        n = self.current_trial['runs']

        DIR = constants.OPTIMIZATION_OUTPUT_PATH + '/bayes_feed_2024-12-30_01-12-05'
        FINAL_CSV_PATH = os.path.join(DIR, 'final_results.csv')
        df = pd.read_csv(FINAL_CSV_PATH)

        max_score = df['score'].max()
        best_row = df[df['score'] == max_score]

        best_metric_path = os.path.join(DIR, 'metrics/metric_41.pickle')
        best_metric = self.cb.load_metric(best_metric_path)
        score = self.score_method(best_metric)

        best_params = ControllerParameters()
        for critic in constants.DEFAULT_MPPI_CRITIC_NAMES:
            best_params.set_critic_weight(
                critic, best_row.iloc[0][f'{critic}.cost_weight'])

        # get row of max score
        self.test_params = deepcopy(best_params)
        self.cb.update_parameters(self.test_params)

        for i in range(1, n + 1):
            # do not change parameters
            yield i

    def generator_random(self):
        n = self.current_trial['runs']

        # generate random parameters and update the controller benchmark
        for i in range(1, n + 1):
            self.test_params.randomize_weights()
            self.cb.update_parameters(self.test_params)
            yield i

    def generator_grid(self):
        n = 101
        scale = 1
        # generate grid and update the controller benchmark
        i = 1
        grid_space = np.arange(0, n, scale)

        for critic in constants.DEFAULT_MPPI_CRITIC_NAMES:
            # load back the default parameters
            self.test_params = deepcopy(self.reference_params)
            for v in grid_space:
                self.test_params.set_critic_weight(critic, v)
                self.cb.update_parameters(self.test_params)
                yield i
                i += 1

    def generator_bayesian(self):
        space = [Real(0.0, 100.0, name=critic)
                 for critic in constants.DEFAULT_MPPI_CRITIC_NAMES]
        optimizer = Optimizer(dimensions=space, random_state=0)

        n = self.current_trial['runs']
        for i in range(1, n + 1):
            # suggest new parameters using the Optimizer
            suggested = optimizer.ask()
            suggested_params = {dim.name: val for dim, val in zip(space, suggested)}

            # update params
            for critic in constants.DEFAULT_MPPI_CRITIC_NAMES:
                self.test_params.set_critic_weight(critic, suggested_params[critic])
            self.cb.update_parameters(self.test_params)

            # simulate a score using the suggested parameters
            yield i

            # tell the optimizer the result of the evaluation
            optimizer.tell(suggested, self.current_score)

    def get_best_metric(self):
        BEST_ROW = 211
        BEST_ROW += 2
        BEST_ID = 264
        DIR = os.path.join(
            constants.OPTIMIZATION_OUTPUT_PATH,
            'waffle_random_big_2024-12-28_15-49-04')
        FINAL_CSV_PATH = os.path.join(DIR, 'final_results.csv')

        header = pd.read_csv(FINAL_CSV_PATH, nrows=0)
        best_row = pd.read_csv(
            FINAL_CSV_PATH, skiprows=BEST_ROW - 1, nrows=1, header=None)
        best_row.columns = header.columns

        # load metric and get pose
        best_metric_path = os.path.join(DIR, f'metrics/metric_{BEST_ID}.pickle')
        best_metric = self.cb.load_metric(best_metric_path)

        best_params = ControllerParameters()
        for critic in constants.DEFAULT_MPPI_CRITIC_NAMES:
            best_params.set_critic_weight(
                critic, best_row.iloc[0][f'{critic}.cost_weight'])

        return best_metric, best_params

    def generator_bayesian_best(self):
        space = [Real(0.0, 100.0, name=critic)
                 for critic in constants.DEFAULT_MPPI_CRITIC_NAMES]
        optimizer = Optimizer(dimensions=space, random_state=0)

        best_metric, best_params = self.get_best_metric()
        score = self.score_method(best_metric)

        # get position
        position = []
        for critic in constants.DEFAULT_MPPI_CRITIC_NAMES:
            # position.append(best_row.iloc[0][f'{critic}.cost_weight'])
            position.append(best_params.get_critic_weight(critic))

        # add initial position
        optimizer.tell(position, score)

        n = self.current_trial['runs']
        for i in range(1, n + 1):
            # suggest new parameters using the Optimizer
            suggested = optimizer.ask()
            suggested_params = {dim.name: val for dim, val in zip(space, suggested)}

            # update params
            for critic in constants.DEFAULT_MPPI_CRITIC_NAMES:
                self.test_params.set_critic_weight(critic, suggested_params[critic])
            self.cb.update_parameters(self.test_params)

            # simulate a score using the suggested parameters
            yield i

            # tell the optimizer the result of the evaluation
            optimizer.tell(suggested, self.current_score)

    def generator_bayesian_feed(self):
        space = [Real(0.0, 100.0, name=critic)
                 for critic in constants.DEFAULT_MPPI_CRITIC_NAMES]
        optimizer = Optimizer(dimensions=space, random_state=0)
        DIR = os.path.join(
            constants.OPTIMIZATION_OUTPUT_PATH,
            'waffle_random_big_2024-12-28_15-49-04')
        POSITIONS_SCORES_PATH = os.path.join(DIR, 'positions_scores.csv')
        df = pd.read_csv(POSITIONS_SCORES_PATH)

        for i, row in df.iterrows():
            position = eval(row['position'])  # Convert string representation of list back to list
            score = row['score']

            self.logger.info(f"Feeding optimizer: {i} - score: {score}")
            optimizer.tell(position, score)

        n = self.current_trial['runs']
        for i in range(1, n + 1):
            # suggest new parameters using the Optimizer
            suggested = optimizer.ask()
            suggested_params = {dim.name: val for dim, val in zip(space, suggested)}

            # update params
            for critic in constants.DEFAULT_MPPI_CRITIC_NAMES:
                self.test_params.set_critic_weight(critic, suggested_params[critic])
            self.cb.update_parameters(self.test_params)

            # simulate a score using the suggested parameters
            yield i

            # tell the optimizer the result of the evaluation
            optimizer.tell(suggested, self.current_score)

    def score_default(self, metric: ControllerMetric) -> float:
        # smaller is better
        max_time = 60.0
        max_cost = constants.OCCUPANCY_WALL
        max_distance = np.linalg.norm(metric.path_xy[-1] - metric.path_xy[0]) / 2
        max_angle = 180  # degrees

        weight_time = 0.35
        weight_cost = 0.35
        weight_f = 0.3
        weight_distance = 0.25
        weight_angle = 0.25

        normalized_time_elapsed = metric.time_elapsed / max_time
        normalized_avg_cost = metric.avg_cost / max_cost
        normalized_frechet_distance = metric.frechet_distance / max_distance
        normalized_distance_to_goal = metric.distance_to_goal / max_distance
        normalized_angle_to_goal = metric.angle_to_goal / max_angle

        score = (
            weight_time * normalized_time_elapsed +
            weight_cost * normalized_avg_cost +
            weight_f * normalized_frechet_distance +
            weight_distance * normalized_distance_to_goal +
            weight_angle * normalized_angle_to_goal
        )

        return float(score)

    def score_avarage(self, metric: ControllerMetric) -> float:
        # SMALLER IS BETTER
        # avarage values got from distribution
        avg_time = 35.0
        avg_frechet = 0.22
        avg_cost = 11.0
        avg_distance = 0.22
        avg_angle = 0.11

        # max lets be 2 times the average
        max_time = 65.0
        max_frechet = 0.62
        max_cost = 13
        max_distance = 0.3
        max_angle = 0.25

        norm_time = metric.time_elapsed / max_time
        norm_frechet = metric.frechet_distance / max_frechet
        norm_cost = metric.avg_cost / max_cost
        norm_distance = metric.distance_to_goal / max_distance
        norm_angle = metric.angle_to_goal / max_angle

        score = (norm_time**2 + norm_frechet**2 + norm_cost **
                 2 + norm_distance**2 + norm_angle**2)**0.5
        return float(score)

    def score_frechet(self, metric: ControllerMetric) -> float:
        max_distance = np.linalg.norm(metric.path_xy[-1] - metric.path_xy[0]) / 2
        normalized_frechet_distance = metric.frechet_distance / max_distance

        return float(normalized_frechet_distance)

    def score_time(self, metric: ControllerMetric) -> float:
        max_time = 60.0
        normalized_time_elapsed = metric.time_elapsed / max_time

        return float(normalized_time_elapsed)

    def score_jerk(self, metric: ControllerMetric) -> float:
        max_linear_jerk = 1.0
        max_angular_jerk = 5.0
        normalized_rms_linear_jerk = metric.rms_linear_jerk / max_linear_jerk
        normalized_rms_angular_jerk = metric.rms_angular_jerk / max_angular_jerk

        return float(normalized_rms_linear_jerk + normalized_rms_angular_jerk)

    def _run(self, run_uid: str):
        metric = self.cb.run_benchmark(run_uid=run_uid)

        if not metric.success:
            return RunResult(
                id=run_uid,
                success=False,
                time_elapsed=metric.time_elapsed,
                status_msg=metric.status_msg,
            )

        metric_path = self.cb.save_metric(metric)
        score = self.score_method(metric)

        return RunResult(
            id=run_uid,
            success=True,
            time_elapsed=metric.time_elapsed,
            score=score,
            frechet_distance=metric.frechet_distance,
            distance_to_goal=metric.distance_to_goal,
            angle_to_goal=metric.angle_to_goal,
            avg_cost=metric.avg_cost,
            rms_linear_jerk=metric.rms_linear_jerk,
            rms_angular_jerk=metric.rms_angular_jerk,
            metric_path=metric_path,
        )

    @timing_decorator(
        lambda self: self.logger.info('Running optimizer REFERENCE...'),
        lambda self, ex_time: self.logger.info(f'Optimizer finished in {ex_time} s.'))
    def run_reference(self) -> bool:
        self.logger.info("Running reference benchmark")

        # setup
        self.cb.update_parameters(self.reference_params)
        self.cb.update_map(self.current_trial['map'])

        # 120 seconds timeout should be enough for the reference run
        if (self.current_trial['generator'] == 'bayesian_best' or
                self.current_trial['generator'] == 'best' or
                self.current_trial['generator'] == 'best_best'):
            metric, self.reference_params = self.get_best_metric()
        else:
            metric: ControllerMetric = self.cb.run_benchmark(
                run_uid='reference', timeout=120)
            if metric.success is False:
                self.logger.error(
                    f"Failed to run reference benchmark: {metric.status_msg}")
                return False

        self.reference_metric = metric
        self.reference_metric_path = self.cb.save_metric(self.reference_metric)
        self.reference_score = self.score_method(self.reference_metric)
        self.current_score = self.reference_score

        ref_run_result = RunResult(
            id='reference',
            success=True,
            time_elapsed=metric.time_elapsed,
            score=self.reference_score,
            frechet_distance=self.reference_metric.frechet_distance,
            distance_to_goal=self.reference_metric.distance_to_goal,
            angle_to_goal=self.reference_metric.angle_to_goal,
            avg_cost=self.reference_metric.avg_cost,
            rms_linear_jerk=self.reference_metric.rms_linear_jerk,
            rms_angular_jerk=self.reference_metric.rms_angular_jerk,
            metric_path=self.reference_metric_path,
        )
        ref_row = {**ref_run_result, **self.reference_params.to_dict()}
        self.final_output_rows.append(ref_row)
        pd.DataFrame([ref_row]).to_csv(
            self.output_csv_path, mode='a', header=True, index=False)

        self.logger.info(f"Reference benchmark finished with score: {self.reference_score}")

    @timing_decorator(
        lambda self: self.logger.info('Running optimizer..'),
        lambda self, ex_time: self.logger.info(f'Optimizer finished in {ex_time} s.'))
    def run(self) -> None:
        if self.current_trial == {}:
            raise ValueError("Trial not set.")

        if self.generator is None:
            raise ValueError("Generator not set.")

        if self.score_method is None:
            raise ValueError("Score method not set.")

        if self.work_dir == '':
            raise ValueError("Work directory not set.")

        trial_name = self.current_trial['name']
        self.logger.info(f'Updating map: {self.current_trial["map"]}')
        self.cb.update_map(self.current_trial['map'])

        self.logger.debug(f"Running optimizer for trial: {trial_name}")

        best_score = 1e9
        best_params: ControllerParameters = deepcopy(self.test_params)
        best_metric_path = 'None'

        num_trials = self.current_trial['runs']
        successful_trials = 0
        loop_start_time = time.time()
        try:
            for i in self.generator():
                self.logger.info(f"Running trial {trial_name}: {i}/{num_trials}")

                run_result: RunResult = self._run(i)
                if run_result['success'] is False:
                    self.logger.warn(f"Failed to run trial {i}/{num_trials}")
                    # TODO: log this also
                    continue

                successful_trials += 1

                score = run_result['score']
                self.current_score = score
                self.logger.info(f"Trial {i}/{num_trials} finished with score: {score}")
                if run_result['score'] < best_score:
                    best_score = score
                    best_params = deepcopy(self.test_params)
                    best_metric_path = run_result['metric_path']

                new_row = {**run_result, **self.test_params.to_dict()}
                self.final_output_rows.append(new_row)
                pd.DataFrame([new_row]).to_csv(
                    self.output_csv_path, mode='a', header=False, index=False)
        except KeyboardInterrupt:
            self.logger.info("Interrupted by user.")
        except Exception as e:
            self.logger.error(f"Failed to run trial: {e}")
            traceback.print_exc()
        finally:
            pd.DataFrame(self.final_output_rows).to_csv(
                self.output_final_csv_path, header=True, index=False)

            self.logger.info("Optimizer finished.")
            self.logger.info(f"Saving {trial_name} summary to {self.output_yaml_path}")
            with open(self.output_yaml_path, 'w', encoding='utf-8') as file:
                run_summary = {
                    'num_of_trials': num_trials,
                    'successful_trials': successful_trials,
                    'loop_time': time.time() - loop_start_time,
                    'best_score': best_score,
                    'best_params': best_params.to_dict(),
                    'best_metric_path': best_metric_path,
                    'reference_score': self.reference_score,
                    'reference_params': self.reference_params.to_dict(),
                    'reference_metric_path': self.reference_metric_path}

                yaml.dump(
                    {**self.current_trial, **run_summary},
                    file, default_flow_style=False, sort_keys=False)
            self.cb.stop_nodes()
