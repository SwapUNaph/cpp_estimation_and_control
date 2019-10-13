import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt

my_data = genfromtxt('measurements.csv', delimiter=',')

plt.plot(my_data[1:,0], my_data[1:,2], c='red', ls='--')
plt.plot(my_data[1:,0], my_data[1:,4], c='blue')
plt.scatter(my_data[1:,0], my_data[1:,3], c='green')

plt.legend(["Actual State", "Filtered", "Measurements"])
plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")
plt.show()
