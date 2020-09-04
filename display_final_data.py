import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')

x_vals = []
y_vals = []

index = count()

# import os
# os.remove("data.csv")

a_file = open("data.csv", "r")
lines = a_file.readlines()
a_file.close()

filtered_file = open("filtered.csv", "w")
for line in lines:
    if "," in line:
        filtered_file.write(line)

filtered_file.close()

data = pd.read_csv('filtered.csv')
x =  data.iloc[:, 0] 
y1 =  data.iloc[:, 1]
y2 =  data.iloc[:, 2]

plt.cla()

plt.plot(x, y1, label='Channel 1')
plt.plot(x, y2, label='Channel 2')

plt.legend(loc='upper left')
plt.tight_layout()


plt.show()
