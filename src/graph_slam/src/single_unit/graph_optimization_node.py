#!/usr/bin/env python
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy import sparse
from scipy.sparse.linalg import inv, spsolve
from car_msg.msg import Node, Edge, Optimized_Node, Node_List, Edge_List
from std_msgs.msg import Bool

# from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
# import csv
# import tf
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import JointState # For subscrbing JointState to compute odom
# from sensor_msgs.msg import LaserScan # For subscribing Laser Scan
# from sklearn.neighbors import NearestNeighbors # For ICP


# -------------- CLASS --------------


class PoseGraph():

    def __init__(self):
        
        '''(Done)
        To impelement this graph slam optimizer, please follow the steps below:
        1. Use the "create_zero_constructor" function with input: 
            - node_data_set : 5xn array
            - edge_data_set : 4xn array

        2. Use the "optimize" function with input:
            - num_iteration : Max iteration times, default = 10

        3. After the iteration, the result is recorded in "self.node".
        -----------------------------------------------------------------------
        Parameter:
            * node : Pose nodes in graph
            * edge : Edge in graph
            * H    : Information matrix
            * b    : Information vector
        '''
        return
        

    def create_zero_constructor(self, node_set, edge_set):

        '''(Done)
        Create zeros constructor of node and edge.
        ------------------------------------------
        
        (5xn array)
        self.node = [
             id |     |  id >> index                        : int
              x |     |   x >> pose of x                    : float
              y | ... |   y >> pose of y                    : float
            yaw |     | yaw >> pose of yaw                  : float
             rt |     |  rt >> transformation  : 3x3 array  : float
        ]

        (4xn array)
        self.edge = [
            id_from |     | >> index                               : int
              id_to | ... | >> index                               : int
               mean |     | >> relative transformation : 3x1 array : float
               infm |     | >> covariance information  : 3x3 array : float
        ]
        '''

        self.length_node = len(node_set)
        self.length_edge = len(edge_set)
        
        self.node = []
        self.edge = []
        # self.node = np.zeros(np.shape(node_set))
        # self.edge = np.zeros(np.shape(edge_set))

        self.read_data(node_set, edge_set)
        return

    def read_data(self, node_set, edge_set):

        '''(Done)
        Read the node and edge data from the files.
        -------------------------------------------
        node_set: array of node data :4xN
              [0]:  id (list)
              [1]:   x (list)
              [2]:   y (list)
              [3]: yaw (list)

        edge_set: array of edge data :11xN
              [0]: id_from (list)
              [1]:   id_to (list)
            [2~4]:    mean (list)
           [5~10]:    infm (list)
        '''
        # put in node data.
        # node -> list[list1, list2, list3, ..., listN]
        for i_node in range(len(node_set)):
            
            self.node.append([
                int(node_set[i_node][0]),
                float(node_set[i_node][1]),
                float(node_set[i_node][2]),
                float(node_set[i_node][3])
            ])

        
        # -- The element of node[4,:] seems not being used.
        # for i in range(np.size(node_set,1)):
        #     self.node[4,i] = np.array([
        #         [cos(node_set[3,i]), -sin(node_set[3,i]), node_set[1,i]],
        #         [sin(node_set[3,i]),  cos(node_set[3,i]), node_set[2,i]],
        #         [                    0,                      0,             1]
        #     ]) 

        # put in edge data
        # edge -> list[list1, list2, list3, ..., listN]
        for i_edge in range(len(edge_set)):
            self.edge.append([
                int(edge_set[i_edge][0]),
                int(edge_set[i_edge][1]),
                edge_set[i_edge][2],
                edge_set[i_edge][3]
            ])

        return

    def optimize(self, num_iteration=20):

        '''(Done)
        Implement optimization to find a best solution for the graph.
        Optimization will stop when maximal iteration is reached.
        '''
        if num_iteration > 0:
            for i in range(num_iteration):
                print("No. %d iteration of optimization ..." %(i+1))
                self.iterate_graph_slam()
                print("No. %d iteration of optimization finished" %(i+1))
                print("=========================================")
            print("Done Graph Optimization !!")
            return  

        elif num_iteration < 0:
            print("Iteration Setting Error.")
            return
        else:
            print("No Iteration Process.")
            return

    def iterate_graph_slam(self):
        
        '''(Done)
        Iteration of pose graph optimization
        Details of the matrice below refer to paper "A Tutorial on Graph-Based SLAM."
        H : 3n x 3n matrix
        b : 3n x 1  matrix
        '''

        print("Iteration ...")

        print("Linearization ...")
        self.linearize_err_fcn()

        print("Solve the linear system ...")
        self.solve_lin_sys()
            
        return
        
    def linearize_err_fcn(self):
        
        '''(Done)
        Linearize error functions and formulate a linear system
        '''

        # Create zero constructors of H and b 
        self.H = np.zeros( (3*self.length_node  , 3*self.length_node) )
        self.b = np.zeros( (3*self.length_node  , 1) )


        for i_edge in range( self.length_edge ):
            # No. i constraint
            ei = self.edge[i_edge]

            # i_node: id_from
            # j_node: id_to
            i_node  = ei[0]
            j_node  = ei[1]
            
            # T_z: Transformation Matrix
            T_z = self.v2t(ei[2])   #T_B->A

            # omega: Convariance
            omega = ei[3]

            # v_i: pose of node i : x, y, yaw
            # v_j: pose of node j : x, y, yaw
            v_i = np.array([
                [ self.node[i_node][1] ],
                [ self.node[i_node][2] ],
                [ self.node[i_node][3] ] 
            ])
            v_j = np.array([
                [ self.node[j_node][1] ],
                [ self.node[j_node][2] ],
                [ self.node[j_node][3] ]
            ])
            

            # Construct transformation from node to global frame
            T_i = self.v2t(v_i)     #T_A->G
            T_j = self.v2t(v_j)     #T_B->G

            R_i = T_i[0:2, 0:2]
            R_z = T_z[0:2, 0:2]

            # 1st order Derivative 
            si = math.sin(v_i[2])
            ci = math.cos(v_i[2])
            dR_i = np.array([
                [-si,  ci],
                [-ci, -si]
                            ]).transpose()
            
            dt_ij = v_j[0:2] - v_i[0:2]

            # Calculation of Jacobians
            # A: 3x3
            # B: 3x3

            A = np.hstack([ np.dot( -R_z.transpose(), R_i.transpose() ) , np.dot( np.dot( R_z.transpose(), dR_i.transpose() ), dt_ij ) ])
            A = np.vstack([ A, np.array([0,0,-1]) ])

            B = np.hstack([ np.dot( R_z.transpose(), R_i.transpose() ), np.array([[0],[0]]) ])
            B = np.vstack([ B, np.array([0,0,1]) ])

            # Calculation of error vector
            e = self.t2v( np.linalg.inv(T_z).dot( np.linalg.inv(T_i) ).dot(T_j) )
            
            # Formulation of updated data of H & b
            # H_ii: 3x3  | H_ij: 3x3
            # -----------|------------
            # H_ji: 3x3  | H_jj: 3x3
            # b_i:  3x1
            # b_j:  3x1
            H_ii =  A.transpose().dot(omega).dot(A)
            H_ij =  A.transpose().dot(omega).dot(B)
            H_jj =  B.transpose().dot(omega).dot(B)
            b_i  =  -A.transpose().dot(omega).dot(e)
            b_j  =  -B.transpose().dot(omega).dot(e)

            # Index of updated data 
            i_ind_start, i_ind_end = self.id2ind(i_node)
            j_ind_start, j_ind_end = self.id2ind(j_node)
            
            # Update H and b matrix
            self.H[i_ind_start : i_ind_end , i_ind_start : i_ind_end] = self.H[i_ind_start : i_ind_end , i_ind_start : i_ind_end] + H_ii
            self.H[i_ind_start : i_ind_end , j_ind_start : j_ind_end] = self.H[i_ind_start : i_ind_end , j_ind_start : j_ind_end] + H_ij
            self.H[j_ind_start : j_ind_end , i_ind_start : i_ind_end] = self.H[j_ind_start : j_ind_end , i_ind_start : i_ind_end] + H_ij.transpose()
            self.H[j_ind_start : j_ind_end , j_ind_start : j_ind_end] = self.H[j_ind_start : j_ind_end , j_ind_start : j_ind_end] + H_jj
            
            self.b[i_ind_start : i_ind_end] = self.b[i_ind_start : i_ind_end] + b_i
            self.b[j_ind_start : j_ind_end] = self.b[j_ind_start : j_ind_end] + b_j
          
        return

    def solve_lin_sys(self):
        
        '''(Done)
        Solves the linear system and update all pose nodes.
        The system Hx = b is obtained only from relative constraints.
        H is not full rank.
        We solve this by anchoring the position of the 1st vertex
        This can be expressed by adding the equation
        dx(1:3,1) = 0
        which is equivalent to the following
        '''
        
        self.H[0:3, 0:3] = self.H[0:3, 0:3] + np.eye(3)
        H_sparse = sparse.csc_matrix(self.H)
        dx = spsolve(H_sparse, self.b)
        dpose = np.reshape(dx, (3, self.length_node), order='F')
        

        for i_node in range(self.length_node):
            for n in range(len(dpose)):
                self.node[i_node][n+1] = self.node[i_node][n+1] + dpose[n, i_node]

        return

    def v2t(self, vector):
        
        '''(Done)
        vector to homogeneous transformation
        From local to global
        [              |
              Rotaion  | Translation
          _____________|____________
             0   |   0 |      1
        ]
        '''
        c = math.cos(vector[2])
        s = math.sin(vector[2])
        x = float(vector[0])
        y = float(vector[1])
        T = np.array([
            [c,  -s,  x],
            [s,   c,  y],
            [0,   0,  1]
        ])
        return T

    def id2ind(self, id):
        
        '''(Done)
        Converts id to indices in H and b
        '''
        ind_start = 3*id
        ind_end   = 3*id + 3
        return ind_start, ind_end

    def t2v(self, T):
        
        '''(Done)
        homogeneous transformation to vector
        '''
        v = np.zeros((3,1))
        v[0:2, 0] = T[0:2, 2]
        v[2, 0] = math.atan2(T[1,0], T[0,0])
        return v

class DATA_COLLECTOR():
    
    def __init__(self, topic_NODE, topic_EDGE, topic_GRAPH_Request):

        self.node_set = []
        self.edge_set = []


        # Publisher Declaration
        self.optimized_node_pub = rospy.Publisher("/solamr_1/optimized_node", Optimized_Node, queue_size=10)

        # Subscriber Declaration
        rospy.Subscriber(topic_NODE, Node_List, self._get_node)
        rospy.Subscriber(topic_EDGE, Edge_List, self._get_edge)
        rospy.Subscriber(topic_GRAPH_Request, Bool, self._update_node)

    def _get_node(self, node_data):
        self.node_set = []
        for i in range(node_data.Number_Node):

            self.node_set.append([
                node_data.Node_ID[i],
                node_data.global_pose_x[i],
                node_data.global_pose_y[i],
                node_data.global_pose_yaw[i]
            ])

    def _get_edge(self, edge_data):
        
        # cov = np.reshape(np.array(edge_data.covariance), (edge_data.covariance_shape.row, edge_data.covariance_shape.row))
        
        self.edge_set = []
        for i in range(edge_data.Number_Edge):
            
            if abs(edge_data.Node_ID_From[i] - edge_data.Node_ID_To[i])>1:
                cov = np.array([[500,0,0],[0,500,0],[0,0,1000]])
            else:
                cov = np.array([[50,0,0],[0,50,0],[0,0,10]])

            self.edge_set.append([
                edge_data.Node_ID_From[i],
                edge_data.Node_ID_To[i],
                [   
                    edge_data.relative_pose_x[i],
                    edge_data.relative_pose_y[i],
                    edge_data.relative_pose_yaw[i]
                ],
                cov
            ])
    
    def _update_node(self, request):
        
        # The request must be true, and node_set and edge_set cannot be empty.
        if request.data is True and self.node_set and self.edge_set:
            
            # print(self.node_set[-1])
            # print(self.edge_set[-1])


            graph_result = _do_graph_optimization(self.node_set, self.edge_set)
            optimal_node_set = Optimized_Node()
            optimal_node_set.Node_ID = np.array([ i[0] for i in graph_result ])
            optimal_node_set.Optimized_x = np.array([ i[1] for i in graph_result ])
            optimal_node_set.Optimized_y = np.array([ i[2] for i in graph_result ])
            optimal_node_set.Optimized_yaw = np.array([ i[3] for i in graph_result ])
            print(optimal_node_set)
            self.optimized_node_pub.publish(optimal_node_set)

            print("Mission Completed")
        
        else:
            print("Fail.")
            print("request is ", request.data)
            print(bool(self.node_set))
            print(bool(self.edge_set))
            

# -------------- GLOBAL FUNCTION --------------
def _do_graph_optimization(Node_set, Edge_set, iteration=20):
    GraphOptimization = PoseGraph()
    GraphOptimization.create_zero_constructor(Node_set, Edge_set)
    GraphOptimization.optimize(iteration)

    return GraphOptimization.node

# -------------- MAIN --------------
if __name__ == "__main__":
    rospy.init_node('Graph_Optimization_NODE', anonymous=True)
    rate = rospy.Rate(10)

    try:
        
        info = DATA_COLLECTOR(  topic_NODE = '/solamr_1/collector_node',
                                topic_EDGE = '/solamr_1/collector_edge', 
                                topic_GRAPH_Request = '/solamr_1/graph_request' )
        
        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        pass
        
        