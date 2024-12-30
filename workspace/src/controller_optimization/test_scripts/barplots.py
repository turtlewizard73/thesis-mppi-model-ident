import matplotlib.pyplot as plt
import numpy as np

# Assuming TrialData is a class with attributes for multiple metrics


class TrialData:
    def __init__(self, name, run_time_avg, run_time_std, angle_to_goal_avg, angle_to_goal_std, distance_to_goal_avg, distance_to_goal_std):
        self.name = name
        self.run_time_avg = run_time_avg
        self.run_time_std = run_time_std
        self.angle_to_goal_avg = angle_to_goal_avg
        self.angle_to_goal_std = angle_to_goal_std
        self.distance_to_goal_avg = distance_to_goal_avg
        self.distance_to_goal_std = distance_to_goal_std


# Sample data: Replace these with your actual TrialData instances
trial1 = TrialData("Trial 1", 10.5, 0.8, 5.3, 0.5, 8.2, 0.7)
trial2 = TrialData("Trial 2", 12.3, 1.1, 6.1, 0.6, 9.0, 0.9)
trial3 = TrialData("Trial 3", 9.7, 0.5, 4.8, 0.4, 7.5, 0.6)
trial4 = TrialData("Trial 4", 11.0, 0.9, 5.5, 0.5, 8.3, 0.8)

# Collect data for plotting
trials = [trial1, trial2, trial3, trial4]
labels = [trial.name for trial in trials]
run_time_avg = [trial.run_time_avg for trial in trials]
run_time_std = [trial.run_time_std for trial in trials]
angle_to_goal_avg = [trial.angle_to_goal_avg for trial in trials]
angle_to_goal_std = [trial.angle_to_goal_std for trial in trials]
distance_to_goal_avg = [trial.distance_to_goal_avg for trial in trials]
distance_to_goal_std = [trial.distance_to_goal_std for trial in trials]

# Plotting
x = np.arange(len(labels))  # the label locations
width = 0.35  # bar width

fig, ax1 = plt.subplots(figsize=(10, 6))

# Bars for distance_to_goal_avg
bars1 = ax1.bar(x - width / 2, distance_to_goal_avg, width,
                yerr=distance_to_goal_std, capsize=5, label='Distance to Goal', color='b')
ax1.set_ylabel('Distance [m]', color='b')
ax1.tick_params(axis='y', labelcolor='b')

# Create second y-axis for angle_to_goal_avg
ax2 = ax1.twinx()
bars2 = ax2.bar(x + width / 2, angle_to_goal_avg, width, yerr=angle_to_goal_std,
                capsize=5, label='Angle to Goal', color='g')
ax2.set_ylabel('Angle to Goal [rad]', color='g')
ax2.tick_params(axis='y', labelcolor='g')

# Add labels and title
ax1.set_xlabel('Trials')
ax1.set_title('Distance and Angle to Goal Average with Standard Deviation')
ax1.set_xticks(x)
ax1.set_xticklabels(labels)

# Add legends
fig.legend(loc="upper center", bbox_to_anchor=(0.5, 0.9), ncol=2)

# Add value annotations on bars
for bars in [bars1, bars2]:
    for bar in bars:
        height = bar.get_height()
        ax = ax1 if bar in bars1 else ax2
        ax.annotate(f'{height:.1f}',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3),  # offset for text
                    textcoords="offset points",
                    ha='center', va='bottom')

# Show plot
plt.tight_layout()
plt.show()
