#!/usr/bin/env python
from matplotlib import pyplot as plt
import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
from sklearn.neighbors import NearestNeighbors # For ICP
from nav_msgs.msg import Odometry
import tf


class Viewer:
    """
    Use matplotlib to show costmap and plot cmd_vel predicted locations.
    """
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        plt.ion()
        plt.show()

    def update(self, scan):
        """
        Update matplotlib ax.
        """

        self._plot_raw_data(scan)
        plt.pause(0.01)
        self.fig.canvas.draw()
        


    

    def _plot_raw_data(self, scan):

        global MAP_1_list, MAP_2_list, MAP_3_list 

        map_1 = np.array(MAP_1_list)
        map_2 = np.array(MAP_2_list)
        map_3 = np.array(MAP_3_list)
        self.ax.cla() 
        self.ax.scatter(map_1[:,0], map_1[:,1], c='black', s=2, label="scan_1")
        self.ax.scatter(map_2[:,0], map_2[:,1], c='red', s=2, label="scan_2")
        self.ax.scatter(scan[:,0], scan[:,1], c='blue', s=2, label="scan_2")
        self.ax.grid(which='both', color='grey')

        MAP_2_list = []


    


"""
Test
"""
# import math
# import numpy as np
# from sklearn.neighbors import NearestNeighbors


def euclidean_distance(point1, point2):
    """
    Euclidean distance between two points.
    :param point1: the first point as a tuple (a_1, a_2, ..., a_n)
    :param point2: the second point as a tuple (b_1, b_2, ..., b_n)
    :return: the Euclidean distance
    """
    a = np.array(point1)
    b = np.array(point2)

    return np.linalg.norm(a - b, ord=2)


def point_based_matching(point_pairs):
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


def icp(reference_points, points, max_iterations=50, distance_threshold=0.5, convergence_translation_threshold=1e-3,
        convergence_rotation_threshold=1e-2, point_pairs_threshold=10, verbose=False):
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
        closest_rot_angle, closest_translation_x, closest_translation_y = point_based_matching(closest_point_pairs)
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

    return transformation_history, points, [yaw, x, y]





# --

def t2v(T):
    '''(Done)
    homogeneous transformation to vector
    '''
    v = np.zeros((3,1))
    v[0:2, 0] = T[0:2, 2]
    v[2, 0] = math.atan2(T[0,1], T[0,0])
    return v

def cb_scan_1(msg):

    global MAP_1,  MAP_1_list, MAP_2_list, SIZE ,ROBOT_X, ROBOT_Y, ROBOT_YAW
    center = int(SIZE/2)
    resolution = int(2000 / SIZE)

    MAP_2_list = []
    for ind, dist in enumerate(msg.ranges):
        if dist >= msg.range_min and dist <= msg.range_max:
            
            x = dist * math.cos(ind * math.pi / 725 + ROBOT_YAW) + ROBOT_X
            y = dist * math.sin(ind * math.pi / 725 + ROBOT_YAW) + ROBOT_Y

            px = int(100*x/resolution) + center
            py = int(100*y/resolution) + center
            MAP_1[px,py] = -1
            MAP_1_list.append([
                x,
                y
            ])
            MAP_2_list.append([
                x+0.1*math.cos(ind),
                y-0.1*math.cos(ind)
            ])

def cb_odom(data):

    global ROBOT_X, ROBOT_Y, ROBOT_YAW

    _, _, ROBOT_YAW = tf.transformations.euler_from_quaternion([
                                                                data.pose.pose.orientation.x,
                                                                data.pose.pose.orientation.y,
                                                                data.pose.pose.orientation.z,
                                                                data.pose.pose.orientation.w,
    ])

    ROBOT_X = data.pose.pose.position.x
    ROBOT_Y = data.pose.pose.position.y



def cb_scan_2(msg):


    global  MAP_2,  MAP_2_list, SIZE, MAP_3_list
    center = int(SIZE/2)
    resolution = int(2000 / SIZE)

    for ind, dist in enumerate(msg.ranges):
        if dist >= msg.range_min and dist <= msg.range_max:
            
            # MAP_2_list.append([
            #     ind,
            #     dist
            # ])
            # x = dist * math.cos(ind * math.pi / 725 +(0.3)) #+(-0.5)
            # y = dist * math.sin(ind * math.pi / 725 +(0.3)) #+(-0.5)

            x = dist * math.cos(ind * math.pi / 725 + 2.8) -5 
            y = dist * math.sin(ind * math.pi / 725 + 2.8) -0.5

            px = int(100*x/resolution) + center
            py = int(100*y/resolution) + center
            
            MAP_2[px,py] = -1

            MAP_2_list.append([
                x,
                y
            ])

if __name__ == "__main__":
    
    # -- lookup table
    RAD_TABLE_COS = []
    RAD_TABLE_SIN = []
    for deg in range(360):
        RAD_TABLE_COS.append(
            int(1000*math.cos(deg/360 *math.pi))
        )
        RAD_TABLE_SIN.append(
            int(1000*math.sin(deg/360 *math.pi))
        )

    ROBOT_X, ROBOT_Y, ROBOT_YAW = -4.5,0,2.4

    # -- global vars
    RESOLUTION = 20
    SIZE = int(2000 / RESOLUTION)
    COST = 0
    MAP_1 = np.zeros((SIZE,SIZE))
    MAP_2 = np.zeros((SIZE,SIZE))
    MAP_1_list = []
    MAP_2_list = []
    MAP_3_list = []
    # -- ros node and params
    rospy.init_node(name="viewer", anonymous=False)
    
    # -- ros function
    rospy.Subscriber(name='/solamr_1/scan_lidar', data_class=LaserScan, callback=cb_scan_1)
    # rospy.Subscriber(name='/solamr_2/scan_lidar', data_class=LaserScan, callback=cb_scan_2)
    rospy.Subscriber(name='/solamr_1/odom', data_class=Odometry, callback=cb_odom)


    # -- matplotlib show 
    viewer = Viewer()
    rate = rospy.Rate(hz=2)
    

    while not rospy.is_shutdown():
        
        if MAP_1_list and MAP_2_list :
            
            A = np.array(MAP_1_list)
            B = np.array(MAP_2_list)


            # T,_ = ICP().icp(A, B)
            # print(T)

            T, scan_aligned, sol = icp(A, B)
            

        viewer.update(scan_aligned)
        rate.sleep()
