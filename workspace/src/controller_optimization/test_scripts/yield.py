import random

# Generator function for random values


def random_generator(lower_bound=1, upper_bound=100):
    for _ in range(5):
        yield random.randint(lower_bound, upper_bound)

# Generator function for a pre-set list of values


def preset_list_generator():
    for value in [10, 20, 30, 40, 50]:
        yield value

# Main function to run trials and print test parameters


def run_trials(generator_func):
    print("Starting trials...")
    for i, test_params in enumerate(generator_func(), start=1):
        print(f"Trial {i}: test_params = {test_params}")


# Example usage

# Using the random generator
print("Using Random Generator:")
run_trials(random_generator)

# Using the preset list generator
print("\nUsing Pre-set List Generator:")
run_trials(preset_list_generator)
