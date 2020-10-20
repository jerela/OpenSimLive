from collections import deque
import matplotlib.pyplot as pyplot
import numpy as np

fig = pyplot.figure()
ax = fig.add_subplot(111, autoscale_on="True")
x = deque([0])
y = deque([0])
line1, = ax.plot(x, y, 'r-')
