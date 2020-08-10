#!/usr/bin/env python
# import matplotlib.pyplot as plt
import numpy as np



x = []
for i in range(10):
    x.append([i,i])

print(np.array(x))

# x = np.linspace(0.5 , 3.5 , 100)
# y = np.sin(x)
# y1 = np.random.randn(100)
# i = 0

# a = np.array([0,1,3,4])
# x = np.array([
#     [1,3,5,7,9,11,13,15,17,19],
#     [2,3,4,5,6, 7, 8, 9,10,11]
# ])

# for i, s in enumerate(x.T):
    # print(i,s)

# x = np.linspace(0, 10, 100)
# y = np.cos(x)

# fig = plt.figure()

# for p in range(50):
#     p=3
#     updated_x=x+p
#     updated_y=np.cos(x)
#     plt.plot(updated_x,updated_y)
#     plt.draw()  
#     x=updated_x
#     y=updated_y
#     plt.pause(0.0002)
#     fig.clear()