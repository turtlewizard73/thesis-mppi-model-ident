from discrete_frechet import FastDiscreteFrechetMatrix, euclidean
import numpy as np
import similaritymeasures
import matplotlib.pyplot as plt

from frechet_display import compute_frechet_distance

# Generate random experimental data
x = np.linspace(0, 2 * np.pi, 100)
y = np.sin(x)
exp_data = np.zeros((100, 2))
exp_data[:, 0] = x
exp_data[:, 1] = y

# Generate random numerical data
x = np.linspace(0, 2 * np.pi, 100)
y = np.sin(x) + np.random.rand(100) * 0.1
num_data = np.zeros((100, 2))
num_data[:, 0] = x
num_data[:, 1] = y

# quantify the difference between the two curves using PCM
# pcm = similaritymeasures.pcm(exp_data, num_data)

# quantify the difference between the two curves using
# Discrete Frechet distance
df = similaritymeasures.frechet_dist(exp_data, num_data)
print(df)

df2 = compute_frechet_distance(exp_data, num_data)
print(df2)

frechet_calc = FastDiscreteFrechetMatrix(euclidean)
df3 = frechet_calc.distance(exp_data, num_data)
print(df3)


# quantify the difference between the two curves using
# area between two curves
# area = similaritymeasures.area_between_two_curves(exp_data, num_data)

# quantify the difference between the two curves using
# Curve Length based similarity measure
# cl = similaritymeasures.curve_length_measure(exp_data, num_data)

# quantify the difference between the two curves using
# Dynamic Time Warping distance
# dtw, d = similaritymeasures.dtw(exp_data, num_data)

# mean absolute error
# mae = similaritymeasures.mae(exp_data, num_data)

# mean squared error
# mse = similaritymeasures.mse(exp_data, num_data)

# print the results

# plot the data
plt.figure()
plt.plot(exp_data[:, 0], exp_data[:, 1])
plt.plot(num_data[:, 0], num_data[:, 1])
plt.show()
