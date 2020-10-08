#!/usr/bin/env python
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from car_msg.msg import Node, Edge, Optimized_Node, Node_List, Edge_List
from std_msgs.msg import Bool

# from scipy import sparse
# from scipy.sparse.linalg import inv, spsolve
# from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
# import csv
# import tf
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import JointState # For subscrbing JointState to compute odom
# from sensor_msgs.msg import LaserScan # For subscribing Laser Scan
# from sklearn.neighbors import NearestNeighbors # For ICP


# -------------- CLASS --------------

class Feature():

    def __init__(self):
        pass


    def _tf_polar_to_xy(self, scan_set, range_max=5, range_min=0.05):
        """
        Transfer scan from polar coordinate to xy coordinate.
        """
        xy_set = []
        for ind, dist in enumerate(scan_set):
            if dist >= range_min and dist <= range_max:
                
                x = dist * math.cos(ind*math.pi/725 - math.pi/2)
                y = dist * math.sin(ind*math.pi/725 - math.pi/2)

                xy_set.append([
                    x,y
                ])

        return xy_set

    def _cal_close_consecutive_dist(self, xy_set, dist_threshold=1):
        """
        Calculate the "CLOSE" consecutive distance.
        """
        close_consecutive_dist_set = []
        for ind in range(len(xy_set) - 1):
            del_x = xy_set[ind][0] - xy_set[ind+1][0]
            del_y = xy_set[ind][1] - xy_set[ind+1][1]
            dist = (del_x**2 + del_y**2)**0.5
            
            if dist <= dist_threshold:
                close_consecutive_dist_set.append(
                    dist
                )
        
        close_consecutive_dist_sum = sum(close_consecutive_dist_set)

        return close_consecutive_dist_set, close_consecutive_dist_sum

    def _cal_consecutive_dist(self, xy_set):
        """
        Calculate the consecutive distance.
        """
        consecutive_dist_set = []
        for ind in range(len(xy_set) - 1):
            del_x = xy_set[ind][0] - xy_set[ind+1][0]
            del_y = xy_set[ind][1] - xy_set[ind+1][1]
            dist = (del_x**2 + del_y**2)**0.5
            
            consecutive_dist_set.append(
                dist
            )
        
        consecutive_dist_sum = sum(consecutive_dist_set)

        return consecutive_dist_set, consecutive_dist_sum

    def _cal_curvature(self, consecutive_dist_set):
        """
        Calculate the curvature of the points.
        """
        curvature_set = []
        for ind in range(len(consecutive_dist_set)-2):
            d1 = consecutive_dist_set[ind]
            d2 = consecutive_dist_set[ind+1]
            d3 = consecutive_dist_set[ind+2]

            Area = ((d1+d2+d3)/2 * d1*d2*d3)**0.5

            K = 4*Area / (d1*d2*d3)
            curvature_set.append(
                K
            )

        return curvature_set            

    def _cal_curvature_standard_deviation(self, curvature_set):
        """
        Calculate STD of curvatures.
        """
        return self._cal_standard_deviation(curvature_set)

    def _cal_standard_deviation(self, data_set):
        """
        Calculate STD.
        """
        std = np.std(data_set)

        return std

    def _cal_centroid(self, xy_set):
        """
        Calculate xy centroid.
        """
        xy = np.array(xy_set)
        x_mean = np.mean(xy[:,0])
        y_mean = np.mean(xy[:,1])

        return x_mean, y_mean

    def _cal_geometry_standard_deviation(self, xy_set, x_mean, y_mean):
        """
        Calculate geometry STD: sum(((xi-xm)**2 + (yi-ym)**2)**0.5)/N
        """
        val = 0
        for ind in range(len(xy_set)):
            del_x = xy_set[ind][0] - x_mean
            del_y = xy_set[ind][1] - y_mean
            dist = (del_x**2 + del_y**2)**0.5
            val = val + dist

        std = val / len(xy_set)

        return std

    def _cal_consecutive_dist_standard_deviation(self, consecutive_dist_set):
        """
        Calculate STD of consecutive distances.
        """
        return self._cal_standard_deviation(consecutive_dist_set)

    def _cal_range_standard_deviation(self, scan_set, range_max=5, range_min=0.05):
        """
        Calculate STD of ranges.
        """
        ranges = []
        for ind, dist in enumerate(scan_set):
            if dist >= range_min and dist <= range_max:
                ranges.append(dist)
            
        return self._cal_standard_deviation(ranges)


class DATA_COLLECTOR():
    
    def __init__(self, topic_NODE, topic_EDGE):

        self.node_set = []
        self.edge_set = []


        # Publisher Declaration
        # self.optimized_node_pub = rospy.Publisher("/solamr_1/optimized_node", Optimized_Node, queue_size=10)

        # Subscriber Declaration
        rospy.Subscriber(topic_NODE, Node_List, self._get_node)
        rospy.Subscriber(topic_EDGE, Edge_List, self._get_edge)
        

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
        
        cov = np.array([[50,0,0],[0,50,0],[0,0,10]])
        self.edge_set = []
        for i in range(edge_data.Number_Edge):
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

    def _find_candidate(self):

        length = len(self.node_set)
        if length > 0 :
            self.candidate_list = []
            for i in range(length-1):
                flag = distance_check(
                    [self.node_set[-1][1], self.node_set[-1][2]],
                    [self.node_set[i][1],  self.node_set[i][2]]
                )
                if flag:
                    self.candidate_list.append(
                        self.node_set[i]
                    )
            
            self.candidate_list.append(
                self.node_set[-1]
            )

            return
        
        else:
            return

    

# -------------- GLOBAL FUNCTION --------------


def _cal_feature_vector(scan_in_xy):
        
    feature_vector = []
    xy_set = scan_in_xy
    f = Feature()
    # xy_set = f._tf_polar_to_xy(scan)
    close_consecutive_dist_set, close_consecutive_dist_sum  = f._cal_close_consecutive_dist(xy_set)
    consecutive_dist_set,       consecutive_dist_sum        = f._cal_consecutive_dist(xy_set)
    consecutive_dist_std                                    = f._cal_consecutive_dist_standard_deviation(consecutive_dist_set)
    curvature_set                                           = f._cal_curvature(consecutive_dist_set)
    curvature_std                                           = f._cal_curvature_standard_deviation(curvature_set)
    x_mean,                     y_mean                      = f._cal_centroid(xy_set)
    geometry_std                                            = f._cal_geometry_standard_deviation(xy_set, x_mean, y_mean)
    range_std                                               = f._cal_range_standard_deviation(scan)

    feature_vector.append([
        close_consecutive_dist_sum,
        consecutive_dist_sum,
        consecutive_dist_std,
        curvature_std,
        [x_mean, y_mean],
        geometry_std,
        range_std
    ])

    return np.array(feature_vector).T

def _identify_similarity(feature_vector_A, feature_vector_B, method=0):

    if method == 0:
        return cos_similarity(feature_vector_A, feature_vector_B)
    elif method ==1:
        return correlation(feature_vector_A, feature_vector_B)
    else:
        print("Method Selection Error...")
        return

def distance_check(pointA, pointB, dist_limit=1):

    dist = ( (pointA[0]-pointB[0])**2 + (pointA[1]-pointB[1])**2 )**0.5
    if dist <= dist_limit:
        return True
    else:
        return False

def cos_similarity(vector1, vector2):
    """
    Input 2 vector as following form:
    vector1: Nx1 nd array
    vector2: Nx1 nd array 
    ---------------------------------
    Output the cosine value between these 2 vector as similarity.
    """

    dist1 = np.sum(vector1**2)**0.5
    dist2 = np.sum(vector2**2)**0.5

    dot_value = vector1.T.dot(vector2)
    cos_value = dot_value / (dist1*dist2)

    return cos_value

def correlation(vector1, vector2):
    """
    Input 2 vector as following form:
    vector1: Nx1 nd array
    vector2: Nx1 nd array 
    ---------------------------------
    Output the correlation matrix.
    """

    data = np.hstack((vector1, vector2)).T
    
    return np.corrcoef(data)

# -------------- MAIN --------------
if __name__ == "__main__":
    rospy.init_node('Loop_Closing_NODE', anonymous=True)
    rate = rospy.Rate(10)

    try:
        
        info = DATA_COLLECTOR(  topic_NODE = '/solamr_1/collector_node',
                                topic_EDGE = '/solamr_1/collector_edge', 
                                )
        
        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        pass
        
        