import pandas as pd
import random
import numpy as np

# Function to simulate the random generation of data


def generate_random_data():
    # Randomize data for new_data fields
    reference_score = random.uniform(0, 100)  # Random float score between 0 and 100
    success = random.choice([True, False])  # Random success value
    run_time = random.uniform(10, 300)  # Random run_time between 10 and 300 seconds
    msg = random.choice(['Success', 'Error', 'Timeout', 'Completed'])  # Random message

    # Assume reference_metric and critics are mocked with random values
    critics = [{'name': f'Critic_{i}', 'cost_weight': random.uniform(
        1, 100)} for i in range(5)]  # 5 random critics

    return reference_score, success, run_time, msg, critics


# Initialize list to collect rows
rows = []

# Generate 10 rows of randomized data
for i in range(10):  # Loop to create 10 new rows
    # Get random data
    reference_score, success, run_time, msg, critics = generate_random_data()

    # Generate the new_data dictionary
    new_data = {
        'id': i,  # Use 'i' to create unique IDs
        'score': reference_score,
        'success': success,
        'run_time': run_time,
        'msg': msg
    }

    # Add critic weights in a loop
    for critic in critics:
        new_data[f'{critic["name"]}.weight'] = critic['cost_weight']

    # Collect the new_data dictionary in the rows list
    rows.append(new_data)

# Once all new_data dictionaries are collected, create a DataFrame in one go
output_df = pd.DataFrame(rows)

# Print the resulting DataFrame
print(output_df)
