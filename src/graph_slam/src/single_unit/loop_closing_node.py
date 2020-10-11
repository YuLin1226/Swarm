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
from sklearn.neighbors import NearestNeighbors # For ICP


# -------------- CLASS --------------
class ScanMatching():

    def __init__(self):
        pass


    def euclidean_distance(self, point1, point2):
        """
        Euclidean distance between two points.
        :param point1: the first point as a tuple (a_1, a_2, ..., a_n)
        :param point2: the second point as a tuple (b_1, b_2, ..., b_n)
        :return: the Euclidean distance
        """
        a = np.array(point1)
        b = np.array(point2)

        return np.linalg.norm(a - b, ord=2)


    def point_based_matching(self, point_pairs):
        """
        This function is based on the paper "Robot Pose Estimation in Unknown Environments by Matching 2D Range Scans"
        by F. Lu and E. Milios.

        :param point_pairs: the matched point pairs [((x1, y1), (x1', y1')), ..., ((xi, yi), (xi', yi')), ...]
        :return: the rotation angle and the 2D translation (x, y) to be applied for matching the given pairs of points
        """

        x_mean = 0
        y_mean = 0
        xp_mean = 0
        yp_mean = 0
        n = len(point_pairs)

        if n == 0:
            return None, None, None

        for pair in point_pairs:

            (x, y), (xp, yp) = pair

            x_mean += x
            y_mean += y
            xp_mean += xp
            yp_mean += yp

        x_mean /= n
        y_mean /= n
        xp_mean /= n
        yp_mean /= n

        s_x_xp = 0
        s_y_yp = 0
        s_x_yp = 0
        s_y_xp = 0
        for pair in point_pairs:

            (x, y), (xp, yp) = pair

            s_x_xp += (x - x_mean)*(xp - xp_mean)
            s_y_yp += (y - y_mean)*(yp - yp_mean)
            s_x_yp += (x - x_mean)*(yp - yp_mean)
            s_y_xp += (y - y_mean)*(xp - xp_mean)

        rot_angle = math.atan2(s_x_yp - s_y_xp, s_x_xp + s_y_yp)
        translation_x = xp_mean - (x_mean*math.cos(rot_angle) - y_mean*math.sin(rot_angle))
        translation_y = yp_mean - (x_mean*math.sin(rot_angle) + y_mean*math.cos(rot_angle))

        return rot_angle, translation_x, translation_y


    def icp(self, reference_points, points, max_iterations=100, distance_threshold=0.5, convergence_translation_threshold=1e-5,
            convergence_rotation_threshold=1e-2, point_pairs_threshold=50, verbose=False):
        """
        An implementation of the Iterative Closest Point algorithm that matches a set of M 2D points to another set
        of N 2D (reference) points.

        :param reference_points: the reference point set as a numpy array (N x 2)
        :param points: the point that should be aligned to the reference_points set as a numpy array (M x 2)
        :param max_iterations: the maximum number of iteration to be executed
        :param distance_threshold: the distance threshold between two points in order to be considered as a pair
        :param convergence_translation_threshold: the threshold for the translation parameters (x and y) for the
                                                transformation to be considered converged
        :param convergence_rotation_threshold: the threshold for the rotation angle (in rad) for the transformation
                                                to be considered converged
        :param point_pairs_threshold: the minimum number of point pairs the should exist
        :param verbose: whether to print informative messages about the process (default: False)
        :return: the transformation history as a list of numpy arrays containing the rotation (R) and translation (T)
                transformation in each iteration in the format [R | T] and the aligned points as a numpy array M x 2
        """

        transformation_history = []

        x, y, yaw = 0, 0, 0

        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(reference_points)
        
        for iter_num in range(max_iterations):
            if verbose:
                print('------ iteration', iter_num, '------')

            closest_point_pairs = []  # list of point correspondences for closest point rule

            distances, indices = nbrs.kneighbors(points)
            for nn_index in range(len(distances)):
                if distances[nn_index][0] < distance_threshold:
                    closest_point_pairs.append((points[nn_index], reference_points[indices[nn_index][0]]))

            # if only few point pairs, stop process
            if verbose:
                print('number of pairs found:', len(closest_point_pairs))
            if len(closest_point_pairs) < point_pairs_threshold:
                if verbose:
                    print('No better solution can be found (very few point pairs)!')
                break

            # compute translation and rotation using point correspondences
            closest_rot_angle, closest_translation_x, closest_translation_y = self.point_based_matching(closest_point_pairs)
            if closest_rot_angle is not None:
                if verbose:
                    print('Rotation:', math.degrees(closest_rot_angle), 'degrees')
                    print('Translation:', closest_translation_x, closest_translation_y)
            if closest_rot_angle is None or closest_translation_x is None or closest_translation_y is None:
                if verbose:
                    print('No better solution can be found!')
                break

            # transform 'points' (using the calculated rotation and translation)
            c, s = math.cos(closest_rot_angle), math.sin(closest_rot_angle)
            rot = np.array([[c, -s],
                            [s, c]])
            aligned_points = np.dot(points, rot.T)
            aligned_points[:, 0] += closest_translation_x
            aligned_points[:, 1] += closest_translation_y

            # update 'points' for the next iteration
            points = aligned_points

            # update transformation history
            transformation_history.append(np.vstack((np.hstack( (rot, np.array([[closest_translation_x], [closest_translation_y]]) )), np.array([0,0,1]))))

            yaw += closest_rot_angle
            x += closest_translation_x
            y += closest_translation_y

            # check convergence
            if (abs(closest_rot_angle) < convergence_rotation_threshold) \
                    and (abs(closest_translation_x) < convergence_translation_threshold) \
                    and (abs(closest_translation_y) < convergence_translation_threshold):
                if verbose:
                    print('Converged!')
                break

        return transformation_history, points, [x, y, yaw]



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
    
    def __init__(self, topic_SUB_NODE, topic_SUB_EDGE, topic_PUB_EDGE):

        self.node_set = []
        self.edge_set = []


        # Publisher Declaration
        self.loop_closing_edge_pub = rospy.Publisher(topic_PUB_EDGE, Edge, queue_size=10)

        # Subscriber Declaration
        rospy.Subscriber(topic_SUB_NODE, Node_List, self._get_node)
        rospy.Subscriber(topic_SUB_EDGE, Edge_List, self._get_edge)
        

    def _get_node(self, node_data):
        self.node_set = []
        for i in range(node_data.Number_Node):

            self.node_set.append([
                node_data.Node_ID[i],
                node_data.global_pose_x[i],
                node_data.global_pose_y[i],
                node_data.global_pose_yaw[i],
                np.array((node_data.current_scan[i].point_x, node_data.current_scan[i].point_y)).T                
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
                    if not self.node_set[-1][0] == self.node_set[i][0] + 1:
                        self.candidate_list.append(
                            self.node_set[i]
                        )
            
            self.candidate_list.append(
                self.node_set[-1]
            )
            
            if len(self.candidate_list) > 1:
                return True
            else:
                return False
        
        else:
            return False

    def _do_loop_closing(self):

        flag = self._find_candidate()
        if flag:
            length = len(self.candidate_list)
            current_feature_vector = _cal_feature_vector(self.candidate_list[-1][4])
            
            for i in range(length-1):
                ref_current_vector = _cal_feature_vector(self.candidate_list[i][4])
                similarity_val = _identify_similarity(current_feature_vector, ref_current_vector)
                
                if similarity_val > 0.9:
                    _, _, loop_closing_pose = ScanMatching().icp(self.candidate_list[i][4], self.candidate_list[-1][4])
                    current_edge = Edge()
                    current_edge.Node_ID_From = self.candidate_list[i][0]
                    current_edge.Node_ID_To = self.candidate_list[-1][0]
                    current_edge.relative_pose.x = loop_closing_pose[0]
                    current_edge.relative_pose.y = loop_closing_pose[1]
                    current_edge.relative_pose.yaw = loop_closing_pose[2]
                    self.loop_closing_edge_pub.publish(current_edge) 
                    break
                

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
    # range_std                                               = f._cal_range_standard_deviation(scan)

    feature_vector.append([
        close_consecutive_dist_sum,
        consecutive_dist_sum,
        consecutive_dist_std,
        curvature_std,
        x_mean, 
        y_mean,
        geometry_std
        # range_std
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
    rate = rospy.Rate(0.5)

    try:
        
        info = DATA_COLLECTOR(  topic_SUB_NODE = '/solamr_1/collector_node',
                                topic_SUB_EDGE = '/solamr_1/collector_edge',
                                topic_PUB_EDGE = '/solamr_1/loop_closing_edge', 
                                )
        
        while not rospy.is_shutdown():
            info._do_loop_closing()
            rate.sleep()

        # rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        pass
        
        