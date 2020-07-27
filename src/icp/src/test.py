#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np



x = np.linspace(0.5 , 3.5 , 100)
y = np.sin(x)
y1 = np.random.randn(100)
i = 0
while i < len(x):
    plt.scatter(x[i],y[i])
    i = i + 1
plt.show()