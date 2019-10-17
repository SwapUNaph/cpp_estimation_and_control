import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt

my_data = genfromtxt('measurements.csv', delimiter=',')


plt.plot(my_data[1:,0], my_data[1:,2], c='red', ls='--')
plt.plot(my_data[1:,0], my_data[1:,4], c='blue')
plt.scatter(my_data[1:,0], my_data[1:,3], c='green')

plt.title("Measured and Estimated state")
plt.legend(["Actual State", "Estimated State", "Measurements"])
plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")

fig = plt.figure()
plt.title("Estimation error")
plt.plot(my_data[1:,0], my_data[1:,2] - my_data[1:,4], c='blue')
plt.legend(["Estimation error"])
plt.xlabel("Time (s)")
plt.ylabel("Estimation error (m)")
plt.show()
