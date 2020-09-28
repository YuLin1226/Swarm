# Graph Optimization

- Calculation of error vector:
    
    <!-- $$$$ -->
    ![](https://latex.codecogs.com/svg.latex?e=%20{T^j_i}^{-1}%20\cdot%20{T^i_0}^{-1}%20\cdot%20T^j_0)

    Transformations above are derived from:
    1. ![](http://latex.codecogs.com/gif.latex?\{T^i_0},{T^j_0}\rightarrow) odometry pose.
    <!-- ${T^i_0}$ & ${T^i_0}$ $\rightarrow$  -->
    2. ![](http://latex.codecogs.com/gif.latex?\{T^i_j}\rightarrow)  scan matching.

    Node info:
    1. ![](http://latex.codecogs.com/gif.latex?\i)  : Node from.
    2. ![](http://latex.codecogs.com/gif.latex?\j) : Node to.
