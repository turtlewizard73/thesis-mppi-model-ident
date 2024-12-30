from dataclasses import dataclass, field
from tabulate import tabulate


@dataclass
class TrialData:
    name: str
    runs: int
    successful_trials: int
    loop_time: float

    rate_of_success: float = 0.0

    # 1. Plan following metrics
    run_time: list = field(default_factory=list)
    run_time_avg: float = 0.0
    run_time_std: float = 0.0

    frechet_distance: list = field(default_factory=list)
    frechet_distance_avg: float = 0.0
    frechet_distance_std: float = 0.0

    distance_to_goal: list = field(default_factory=list)
    distance_to_goal_avg: float = 0.0
    distance_to_goal_std: float = 0.0
    angle_to_goal: list = field(default_factory=list)
    angle_to_goal_avg: float = 0.0
    angle_to_goal_std: float = 0.0

    avg_cost: list = field(default_factory=list)
    avg_cost_avg: float = 0.0
    avg_cost_std: float = 0.0

    # 2. Dynamics metrics
    avg_linear_velocity: list = field(default_factory=list)
    avg_linear_velocity_avg: float = 0.0
    avg_linear_velocity_std: float = 0.0

    avg_linear_acceleration: list = field(default_factory=list)
    avg_linear_acceleration_avg: float = 0.0
    avg_linear_acceleration_std: float = 0.0
    avg_angular_acceleration: list = field(default_factory=list)
    avg_angular_acceleration_avg: float = 0.0
    avg_angular_acceleration_std: float = 0.0

    rms_linear_jerk: list = field(default_factory=list)
    rms_linear_jerk_avg: float = 0.0
    rms_linear_jerk_std: float = 0.0
    rms_angular_jerk: list = field(default_factory=list)
    rms_angular_jerk_avg: float = 0.0
    rms_angular_jerk_std: float = 0.0

    def print_trial_data(self) -> str:
        table_data = [
            ['Name', self.name, '-'],
            ['Runs', self.runs, '-'],
            ['Successful Trials', self.successful_trials, '-'],
            ['Loop Time', self.loop_time, '[s]'],
            ['Rate of Success', self.rate_of_success, '-'],

            ['Run Time (avg)', self.run_time_avg, '[s]'],
            ['Run Time (std)', self.run_time_std, '[s]'],

            ['Frechet Distance (avg)', self.frechet_distance_avg, '[m]'],
            ['Frechet Distance (std)', self.frechet_distance_std, '[m]'],

            ['Distance to Goal (avg)', self.distance_to_goal_avg, '[m]'],
            ['Distance to Goal (std)', self.distance_to_goal_std, '[m]'],

            ['Angle to Goal (avg)', self.angle_to_goal_avg, '[rad]'],
            ['Angle to Goal (std)', self.angle_to_goal_std, '[rad]'],

            ['Avg of Costs (avg)', self.avg_cost_avg, '-'],
            ['Avg of Costs (std)', self.avg_cost_std, '-'],

            ['Avg Linear Velocity (avg)', self.avg_linear_velocity_avg, '[m/s]'],
            ['Avg Linear Velocity (std)', self.avg_linear_velocity_std, '[m/s]'],

            ['Avg Linear Acceleration (avg)', self.avg_linear_acceleration_avg, '[m/s²]'],
            ['Avg Linear Acceleration (std)', self.avg_linear_acceleration_std, '[m/s²]'],

            ['Avg Angular Acceleration (avg)', self.avg_angular_acceleration_avg, '[rad/s²]'],
            ['Avg Angular Acceleration (std)', self.avg_angular_acceleration_std, '[rad/s²]'],

            ['RMS Linear Jerk (avg)', self.rms_linear_jerk_avg, '[m/s³]'],
            ['RMS Linear Jerk (std)', self.rms_linear_jerk_std, '[m/s³]'],

            ['RMS Angular Jerk (avg)', self.rms_angular_jerk_avg, '[rad/s³]'],
            ['RMS Angular Jerk (std)', self.rms_angular_jerk_std, '[rad/s³]'],
        ]

        return tabulate(
            table_data, headers=["Attribute", "Value", "Unit"], tablefmt="grid", floatfmt=".4f"
        )
