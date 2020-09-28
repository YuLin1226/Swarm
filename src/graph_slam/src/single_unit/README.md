# Graph Optimization

- Calculation of error vector:
    
    <!-- $$$$ -->
    ![](http://latex.codecogs.com/gif.latex?\\e={T^j_i}^{-1}\cdot{T^i_0}^{-1}\cdotT^j_0})
    Transformations above are derived from:
    1. ![](http://latex.codecogs.com/gif.latex?\\{T^i_0},{T^i_0}\rightarrow) odometry pose.
    <!-- ${T^i_0}$ & ${T^i_0}$ $\rightarrow$  -->
    2. ![](http://latex.codecogs.com/gif.latex?\\{T^i_j}\rightarrow)  scan matching.

    Node info:
    1. i : Node from.
    2. j : Node to.
