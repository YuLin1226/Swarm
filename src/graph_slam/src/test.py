import numpy as np
x = []
y = []

for i in range(10):
    x.append(i*1)
    y.append(i*10)

c = np.array((x,y)).T

print(c**2)