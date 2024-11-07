# pip install scikit-learn

from sklearn.model_selection import GridSearchCV
from sklearn.base import BaseEstimator

param_grid = {
    'velocity_cost_weight': [0.1, 0.5, 1.0, 1.5],
    'obstacle_avoidance_weight': [0.1, 0.3, 0.5, 0.7],
    'temperature': [0.5, 1.0, 1.5, 2.0]
}


def evaluate_mppi_config(params):
    # Example of setting parameters (this depends on your actual MPPI controller setup)
    controller.set_velocity_cost_weight(params['velocity_cost_weight'])
    controller.set_obstacle_avoidance_weight(params['obstacle_avoidance_weight'])
    controller.set_temperature(params['temperature'])

    # Run the simulation and get performance metrics
    performance_score = run_simulation(controller)

    return performance_score  # Lower score may indicate better performance


# Wrap the evaluation function in a custom estimator for compatibility with GridSearchCV
class MPPIControllerOptimizer(BaseEstimator):
    def __init__(self, velocity_cost_weight=0.1, obstacle_avoidance_weight=0.1, temperature=0.5):
        self.velocity_cost_weight = velocity_cost_weight
        self.obstacle_avoidance_weight = obstacle_avoidance_weight
        self.temperature = temperature

    def score(self, X, y=None):
        # Call the evaluation function with the parameters
        params = {
            'velocity_cost_weight': self.velocity_cost_weight,
            'obstacle_avoidance_weight': self.obstacle_avoidance_weight,
            'temperature': self.temperature
        }
        return evaluate_mppi_config(params)


# Instantiate the optimizer and perform the grid search
optimizer = MPPIControllerOptimizer()
grid_search = GridSearchCV(
    optimizer, param_grid=param_grid, scoring='neg_mean_squared_error', cv=1)
grid_search.fit(X=[0])  # Dummy input, as we only need parameter search here

# Get the best parameters
best_params = grid_search.best_params_
print("Best parameters found:", best_params)
