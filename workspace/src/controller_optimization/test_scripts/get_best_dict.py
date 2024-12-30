# Example dictionary
trials_scores: dict[str, float] = {
    "trial1": 0.75,
    "trial2": 0.45,
    "trial3": 0.90,
    "trial4": 0.30
}

# Sorting the dictionary by values (floats) in ascending order
sorted_trials_scores = dict(sorted(trials_scores.items(), key=lambda item: item[1]))

print(sorted_trials_scores)
