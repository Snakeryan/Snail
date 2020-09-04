import random
import numpy as np
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')

x_vals = []
y_vals = []

index = count()


def animate(i):

    a_file = open("data.csv", "r")
    lines = a_file.readlines()
    a_file.close()

    filtered_file = open("filtered.csv", "w")
    valid_lines = 0
    for line in lines:
        if "," in line:
            filtered_file.write(line)
            valid_lines = 1

    
    filtered_file.close()

    if valid_lines != 1:
        data = pd.DataFrame(np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]))
    else:
        data = pd.read_csv('filtered.csv')

    x =  data.iloc[:, 0]
    y1 =  data.iloc[:, 1]
    y2 =  data.iloc[:, 2]

    plt.cla()

    plt.plot(x, y1, label='Channel 1')
    plt.plot(x, y2, label='Channel 2')

    plt.legend(loc='upper left')
    plt.tight_layout()


ani = FuncAnimation(plt.gcf(), animate, interval=1000)

plt.tight_layout()
plt.show()

print("hi")