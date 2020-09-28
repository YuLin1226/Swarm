# Graph Optimization

- Calculation of error vector:
    
    $$e= {T^j_i}^{-1} \cdot {T^i_0}^{-1} \cdot T^j_0$$
      
    Transformations above are derived from:
    1. ${T^i_0}$ & ${T^i_0}$ $\rightarrow$ odometry pose.
    2. ${T^j_i}$ $\rightarrow$ scan matching.

    Node info:
    1. $i$ : Node from.
    2. $j$ : Node to.
