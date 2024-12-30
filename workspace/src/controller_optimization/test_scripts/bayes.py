from skopt import Optimizer
import matplotlib.pyplot as plt
import numpy as np
from skopt import gp_minimize
from skopt.space import Real
from skopt.utils import use_named_args
import random
import time


# def generator():
#     for i in range(10):
#         print(f"before {i}")
#         yield i
#         print(f"after {i}")


# for i in generator():
#     print('FOR DO STUFF')
#     print(i)
#     time.sleep(2)
#     print('FOR DO STOFF AFTER')

# Define a toy simulation function
def run_sim(params):
    """
    Simulate a score based on the input parameters.
    For demonstration purposes, we'll use a simple mathematical function.
    """
    x = params['param1']
    y = params['param2']
    # The goal is to minimize this function
    return (x - 0.5)**2 + (y - 0.7)**2 + random.gauss(0, 0.01)  # Add some noise for realism


# Define the parameter space
space = [
    Real(0.0, 1.0, name='param1'),
    Real(0.0, 1.0, name='param2'),
]

# Define the objective function


@use_named_args(space)
def objective(**params):
    return run_sim(params)


# Custom Bayesian Optimization loop using Optimizer
optimizer = Optimizer(dimensions=space, random_state=0)
params = {'param1': 0.3, 'param2': 0.3}  # Initial guess
custom_results = []
start_time_custom = time.time()

for i in range(50):  # Limiting to 20 iterations for demonstration
    print(f"Custom Iteration {i + 1}")

    # Suggest new parameters using the Optimizer
    suggested = optimizer.ask()
    suggested_params = {dim.name: val for dim, val in zip(space, suggested)}

    # Simulate a score using the suggested parameters
    score = run_sim(suggested_params)

    # Tell the optimizer the result of the evaluation
    optimizer.tell(suggested, score)

    # Log the results
    custom_results.append({'iteration': i + 1, 'params': suggested_params, 'score': score})
    print(f"Params: {suggested_params}, Score: {score}")

custom_time = time.time() - start_time_custom


# Built-in Bayesian Optimization with gp_minimize
start_time_builtin = time.time()
builtin_result = gp_minimize(objective, space, n_calls=50, random_state=42)
builtin_time = time.time() - start_time_builtin

# Compare Results
custom_scores = [r['score'] for r in custom_results]
builtin_scores = builtin_result.func_vals

# Visualization

plt.figure(figsize=(12, 6))

# Custom Optimization Results
plt.plot(range(1, 51), custom_scores, marker='o', label='Custom Bayesian Optimization')

# Built-in Optimization Results
plt.plot(range(1, 51), builtin_scores, marker='x', label='Built-in gp_minimize')

plt.xlabel('Iteration')
plt.ylabel('Score')
plt.title('Comparison of Custom vs Built-in Bayesian Optimization')
plt.legend()
plt.grid()
plt.show()

# Print timing information
print(f"Custom Optimization Time: {custom_time:.2f} seconds")
print(f"Built-in Optimization Time: {builtin_time:.2f} seconds")
