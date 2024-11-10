from typing import TypedDict


class SearchOuputDict(TypedDict):
    # for logging the output of a search in yaml
    type: str
    num_of_runs: int
    successful_runs: int
    failed_runs: int
    loop_time: float  # [s]

    timeout: float  # [s]

    best_score: float
    best_parameters: dict
    best_metrics_path: str

    reference_score: float
    reference_parameters: dict
    reference_metrics_path: str


class OutputRowDict(TypedDict):
    # for outputing the evaluation of a single run
    id: str
    success: bool
    score: float
    time_elapsed: float
    avg_cost: float
    distance_to_goal: float
    angle_to_goal: float
    msg: dict
    metric_path: str
