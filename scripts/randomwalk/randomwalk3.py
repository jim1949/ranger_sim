import matplotlib.pyplot as plt
import numpy as np
import random

def r2walk(T):
    x = np.zeros((T))
    y = np.zeros((T))
    for t in range(0,T):
        walk = random.random()
        if 0 < walk < .25:
            x[t] = x[t-1] + 1
        elif .25 < walk < .5:
            x[t] = x[t-1] - 1
        elif .5 < walk < 0.75:
            y[t] = y[t-1] + 1
        else:
            y[t] = y[t-1] - 1
    return x, y
t=1
print(t)

x, y = r2walk(100)

# create a figure
fig = plt.figure()
# create a plot into the figure
ax = fig.add_subplot(111)
# plot the data
ax.plot(x,y)