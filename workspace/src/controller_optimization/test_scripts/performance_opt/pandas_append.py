import pandas as pd
import numpy as np
import time
import psutil
import os

# Helper function to get memory usage


def get_memory_usage():
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / (1024 ** 2)  # Memory in MB


# Benchmark setup
num_iterations = 10000
num_critics = 5
reference_score = 100
result = type('Result', (object,), {'success': True, 'status_msg': 'OK'})()
reference_metric = type('Metric', (object,), {
    'time_elapsed': 1.23,
    'critics': [type('Critic', (object,), {'name': f'critic_{i}', 'cost_weight': np.random.random()})() for i in range(num_critics)]
})()

# Base DataFrame
output_df = pd.DataFrame()

# Benchmark function


def benchmark_method(method_name, update_func):
    global output_df
    output_df = pd.DataFrame()  # Reset DataFrame
    start_mem = get_memory_usage()
    start_time = time.time()

    for _ in range(num_iterations):
        update_func()

    elapsed_time = time.time() - start_time
    end_mem = get_memory_usage()

    return elapsed_time, end_mem - start_mem

# Method 1: Using .loc


def loc_method():
    global output_df
    new_data = {
        'id': 0,
        'score': reference_score,
        'success': result.success,
        'run_time': reference_metric.time_elapsed,
        'msg': result.status_msg,
    }
    new_data.update({
        f'{critic.name}.weight': critic.cost_weight
        for critic in reference_metric.critics
    })
    output_df.loc[len(output_df)] = new_data


# Method 2: Using pd.concat in batches
rows = []


def concat_method():
    global output_df
    new_data = {
        'id': 0,
        'score': reference_score,
        'success': result.success,
        'run_time': reference_metric.time_elapsed,
        'msg': result.status_msg,
    }
    new_data.update({
        f'{critic.name}.weight': critic.cost_weight
        for critic in reference_metric.critics
    })
    rows.append(new_data)
    if len(rows) >= 1000:  # Batch size
        output_df = pd.concat([output_df, pd.DataFrame(rows)], ignore_index=True)
        rows.clear()


# Run benchmarks
methods = {
    'loc': loc_method,
    'concat': concat_method
}

results = {}
for method_name, func in methods.items():
    elapsed_time, mem_usage = benchmark_method(method_name, func)
    results[method_name] = {
        'time (s)': elapsed_time,
        'memory usage (MB)': mem_usage
    }
    print(f"Method: {method_name}")
    print(f"Time elapsed: {elapsed_time:.4f} seconds")
    print(f"Memory usage: {mem_usage:.4f} MB\n")

# Print results
print("Summary:")
print(pd.DataFrame(results).T)
