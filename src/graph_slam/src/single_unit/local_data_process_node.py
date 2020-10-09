#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan # For subscribing Laser Scan
from sensor_msgs.msg import JointState # For subscrbing JointState to compute odom
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from car_msg.msg import Node, Edge, Optimized_Node, Node_List, Edge_List, Scan
from sklearn.neighbors import NearestNeighbors
import tf


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


class ODOM():
    def __init__(self, topic_sub = '/solamr_1/joint_states', topic_pub = '/solamr_1/wheel_odom'):
        '''
        To realize the drift effect, the odometry information has to be generated by ourself.
        Because the odom topic from gazebo is the "Ground Truth" data. Hence it is without any noise.
        '''
        # rospy.init_node('odometry_construction', anonymous=True)
        
        # Parameters of the hardware
        self.wheel_radius = 0.033
        self.wheel_dist = 0.2

        # Init encoder value
        self.enc_Left = 0
        self.enc_Right = 0

        # List
        self.omega_list_L = [0,0,0]
        self.omega_list_R = [0,0,0]

        # Pose
        self.x = 0
        self.y = 0
        self.yaw = 0

        # rospy
        self.odom_pub = rospy.Publisher(topic_pub, Odometry, queue_size=50)
        rospy.Subscriber(topic_sub, JointState, self.get_encoder)
        # rospy.Subscriber('/solamr_1/imu', Imu, self.get_imu)

    # def get_imu(self, msg):
    #     wz = msg.angular_velocity.z
    #     ax = msg.linear_acceleration.x

    def get_encoder(self, msg):
        dt = 1 / 30.0
        fs = 30.0

        enc_L = msg.position[1]
        enc_R = msg.position[0]

        omega_L = (enc_L - self.enc_Left)  *fs
        omega_R = (enc_R - self.enc_Right) *fs

        self.enc_Left = enc_L
        self.enc_Right = enc_R

        self.omega_list_L.append(omega_L)
        self.omega_list_R.append(omega_R)

        omega_L = self.lowpass_filter(self.omega_list_L)
        omega_R = self.lowpass_filter(self.omega_list_R)

        vx, wz = self.cal_twitst(omega_R, omega_L)

        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt
        self.yaw += wz * dt

        if self.yaw <= 0 or self.yaw >= 2*math.pi:
            self.yaw = self.yaw%(2*math.pi)

        self.publish_odom(vx, wz)

   
    def lowpass_filter(self, data_list):
        
        return ( data_list[-1] + 2 * data_list[-2] + data_list[-3] ) / 4

    def cal_twitst(self, wR, wL):

        vx = (wR + wL)/2 * self.wheel_radius
        wz = (wR - wL) * self.wheel_radius / self.wheel_dist
        
        return vx, wz

    def publish_odom(self, vx, wz):
        current_time = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "solamr_1/odom"
        odom.child_frame_id = "solamr_1/base_footprint"
        odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, wz))

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        self.odom_pub.publish(odom)

class Segment():
    def __init__(self, x, y):
        '''
        Parameters:
        d_thres : for segmentation of the consecutive data points.
        d_break_thres : for segmentation of the breakpoints. 
        
        '''
        self.d_thres = 5
        self.d_break_thres = 1
        self.shape_thres = 0.07
        self.R_thres = 1.5
        
        self.circle_thres = (math.pi/180)*60
        self.corner_thres = 0.7

        self.seg = []
        self.do_segment(x, y)
        self.seg = self.classify_segment_type(self.seg)
        self.seg = self.corner_merger_operation(self.seg)
        self.landmark = self.do_landmark(self.seg)


    def do_segment(self, x, y):
        '''
        Input:
        x: 1xn, ndarray
        y: 1xn, ndarray
        '''

        for i in range(len(x)):
            
            d_sum = 0
            for L in range(len(x) - 1):
                d       = cal_dist( x[0] , y[0] , x[L+1] , y[L+1] )
                d_conti = cal_dist( x[L] , y[L] , x[L+1] , y[L+1] )
                d_sum   = d_sum + d_conti
                
                if (d_sum - d) > self.d_thres or d_conti > self.d_break_thres:
                    
                    # If number of a seg is too less, then it won't form a seg.
                    # 因為相鄰太遠的點也會想要形成seg, 但很多情況下此seg僅有一個元素. 
                    if len(x[0:L+1]) < 5:
                        pass
                    else:
                        self.seg.append([
                            x[0:L+1],
                            y[0:L+1],
                            []
                        ])
                    x = x[L+1 : len(x)]
                    y = y[L+1 : len(y)]

                    break


            if len(x) <= 5:
                self.seg[-1][0] = np.hstack((self.seg[-1][0] , x)) 
                self.seg[-1][1] = np.hstack((self.seg[-1][1] , y))
                break
        
       

    def classify_segment_type(self, seg):
        
        # for loop : "number of row" times
        for i in range( len(seg) ):
            x = seg[i][0]
            y = seg[i][1]

            x0 = x[0]
            y0 = y[0]

            xn = x[-1]
            yn = y[-1]

            xm = np.mean(x)
            ym = np.mean(y)

            _, _, r = cal_circle([x0, xm, xn], [y0, ym, yn])
            
            if r > self.R_thres:
                seg[i][2] = "line"
                
            else:
                seg[i][2] = "circle"
        
        return seg
                
    def corner_merger_operation(self, seg):
        k = 0
        i = -1
        for j in range(len(seg) - 1):
            
            i = i + 1 + k
            k = 0

            if i+1 >= len(seg):
                break

            x1 = seg[i][0]
            y1 = seg[i][1]
            # x1m = np.mean(x1)
            # y1m = np.mean(y1)
            
            x2 = seg[i+1][0]
            y2 = seg[i+1][1]
            # x2m = np.mean(x2)
            # y2m = np.mean(y2)

            val = cal_acos([ x1[-1] - x1[0], y1[-1] - y1[0] ], [ x2[-1] - x2[0], y2[-1] - y2[0] ])

            if seg[i][2] == "circle":
            
                # circle , circle
                if seg[i][2] == seg[i+1][2]:
                    # 合併Operation
                    seg[i][0] = np.hstack(( seg[i][0] , seg[i+1][0] ))
                    seg[i][1] = np.hstack(( seg[i][1] , seg[i+1][1] ))
                    seg.pop(i+1)
                    k = -1

                # circle , line   
                else:                       
                    # 判斷轉角
                    if val > self.corner_thres:
                        k = 0
                        
                    # 合併circle (後者理當是Circle，但是誤判成Line的情況)
                    else:
                        
                        # 合併Operation
                        seg[i][0] = np.hstack(( seg[i][0] , seg[i+1][0] ))
                        seg[i][1] = np.hstack(( seg[i][1] , seg[i+1][1] ))                
                        seg.pop(i+1)
                        k = -1
            # seg[i][2] == "line"    
            else:
                
                # line , line
                if seg[i][2] == seg[i+1][2]:  
                    
                    
                    # 判斷轉角
                    if val > self.corner_thres:  
                        pass
                        
                        
                    # 合併line
                    else:
                        
                        # 合併Operation
                        seg[i][0] = np.hstack(( seg[i][0] , seg[i+1][0] ))
                        seg[i][1] = np.hstack(( seg[i][1] , seg[i+1][1] ))                    
                        seg.pop(i+1)
                        k = -1
                        
                    
                # line , circle
                else:                      
                    
                    
                    # 判斷轉角
                    if val > self.corner_thres:
                        pass
                        
                        
                    # 合併line (後者理當是Line，但是誤判成Circle的情況)
                    else:
                        
                        # 合併Operation
                        seg[i][0] = np.hstack(( seg[i][0] , seg[i+1][0] ))
                        seg[i][1] = np.hstack(( seg[i][1] , seg[i+1][1] ))
                        seg.pop(i+1)
                        k = -1
        return seg

    def do_landmark(self, seg):
        circle_num = 0
        for i in range(len(seg)):
            if seg[i][2] == "circle":
                circle_num = circle_num + 1
            
        corner_num = 0
        for i in range( len(seg) - 1 ):
            
            x1 = seg[i][0][-1]
            y1 = seg[i][1][-1]
            x2 = seg[i+1][0][0]
            y2 = seg[i+1][1][0]

            d = cal_dist(x1,y1,x2,y2)
            if d < self.d_break_thres:
                corner_num = corner_num + 1
            


        landmark =  [   ["corner",  corner_num],
                        ["circle",  circle_num] ]
        return landmark


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
                
                x = dist * math.cos(ind*math.pi/725)
                y = dist * math.sin(ind*math.pi/725)

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



class LiDAR_Association():

    def __init__(self, topic_name='/solamr_1/scan_lidar'):

        global RAD_TABLE_COS, RAD_TABLE_SIN 

        self.landmark = None
        self.scan = None
        self.scan_range_min = None
        self.scan_range_max = None
        self.feature_vector = []

        rospy.Subscriber(topic_name, LaserScan, self.get_lidar)
        
    def get_lidar(self, msg):
        

        
        point = []
        for ind, dist in enumerate(msg.ranges):
            if dist >= msg.range_min and dist <= msg.range_max:
                
                x = dist * math.cos(ind * math.pi / 725 - math.pi/2)
                y = dist * math.sin(ind * math.pi / 725 - math.pi/2)
                # x = dist * RAD_TABLE_COS[ind]
                # y = dist * RAD_TABLE_SIN[ind]
                point.append([
                    x,
                    y
                ])
        self.scan = np.array(point)
        # px = self.scan[:,0]
        # py = self.scan[:,1]
        # -- Segment 這段算很慢...
        # Seg = Segment(px, py)
        self.landmark = 1

        # scan = msg.ranges
        # f = Feature()
        # xy_set = f._tf_polar_to_xy(scan)
        # close_consecutive_dist_set, close_consecutive_dist_sum  = f._cal_close_consecutive_dist(xy_set)
        # consecutive_dist_set,       consecutive_dist_sum        = f._cal_consecutive_dist(xy_set)
        # consecutive_dist_std                                    = f._cal_consecutive_dist_standard_deviation(consecutive_dist_set)
        # curvature_set                                           = f._cal_curvature(consecutive_dist_set)
        # curvature_std                                           = f._cal_curvature_standard_deviation(curvature_set)
        # x_mean,                     y_mean                      = f._cal_centroid(xy_set)
        # geometry_std                                            = f._cal_geometry_standard_deviation(xy_set, x_mean, y_mean)
        # range_std                                               = f._cal_range_standard_deviation(scan)

        # self.feature_vector.append([
        #     close_consecutive_dist_sum,
        #     consecutive_dist_sum,
        #     consecutive_dist_std,
        #     curvature_std,
        #     [x_mean, y_mean],
        #     geometry_std,
        #     range_std
        # ])




# Function Area
# -------------------
def cal_acos(v1, v2):
    '''
    Input:
    v1 : Input vector.
    v2 : Input vector.
    
    Output:
    value : Angle between these two vectors. (unit: rad)
    '''
    d1 = (v1[0]**2 + v1[1]**2)**(0.5)
    d2 = (v2[0]**2 + v2[1]**2)**(0.5)
    # print("d1:%f"%d1)
    # print("d2:%f"%d2)

    value = math.acos( (v1[0]*v2[0]+v1[1]*v2[1])/(d1*d2) )
    return value

def cal_circle(x, y):
    '''
    Input:
    x : List or array which stores 3 data respectively.
    y : List or array which stores 3 data respectively.
    
    Output:
    xc : X coordinate of the center.
    yc : X coordinate of the center.
    R  : Radius of the center.
    '''

    A = np.array([
        [ 2*x[0], 2*y[0], 1 ],
        [ 2*x[1], 2*y[1], 1 ],
        [ 2*x[2], 2*y[2], 1 ]
    ])
    B = np.array([
        [ x[0]**2 + y[0]**2 ],
        [ x[1]**2 + y[1]**2 ],
        [ x[2]**2 + y[2]**2 ]
    ])

    if abs(np.linalg.det(A)) < 0.0001:
        xc = None
        yc = None
        R = 100000
         
    
    else:
        X = np.linalg.solve(A, B)
        xc = X[0]
        yc = X[1]
        R  = ( X[2] + xc**2 + yc**2 )**(0.5)

    return xc, yc, R

def cal_dist(xi, yi, xf, yf):
    
    dx = xi - xf
    dy = yi - yf
    
    return (dx**2 + dy**2)**(0.5)

def t2v(T):
    '''(Done)
    homogeneous transformation to vector
    '''
    v = np.zeros((3,1))
    v[0:2, 0] = T[0:2, 2]
    v[2, 0] = math.atan2(T[0,1], T[0,0])
    return v

def v2t(vector):
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




# Main Class
class CAR():
    def __init__(self, topic_SUB_LiDAR='/solamr_1/scan_lidar', topic_SUB_OPT_NODE="/solamr_1/optimized_node", topic_SUB_LOOPCLOSE_EDGE="/solamr_1/loop_closing_edge"):
        self.lidar = LiDAR_Association(topic_SUB_LiDAR)
        self.odom = ODOM()

        self.updated_node = None
        self.flag_getting_updated = False
        self.flag_getting_loop_closing = False
        rospy.Subscriber(topic_SUB_OPT_NODE, Optimized_Node, self._get_updated_node)
        rospy.Subscriber(topic_SUB_LOOPCLOSE_EDGE, Edge, self._get_loop_closing_edge)

    def _get_updated_node(self, node_msg):
        
        self.flag_getting_updated = True
        self.updated_node = node_msg
        return

    def _get_loop_closing_edge(self, msg):
        
        self.flag_getting_loop_closing = True
        
        return

if __name__ == "__main__":
    rospy.init_node('SLAM_Infomation_Construction', anonymous=True)
    rate = rospy.Rate(2)


    # Global Variable

    RAD_TABLE_COS = []
    RAD_TABLE_SIN = []
    for ind in range(725):
        RAD_TABLE_COS.append(
            math.cos(ind * math.pi / 725)
        )
        RAD_TABLE_SIN.append(
            math.sin(ind * math.pi / 725)
        )
    
    try:
        print("sleep for 5 sec.")
        rospy.sleep(1)
        print("Woke UP.")

        # List Set
        '''
        Node Set
        [0]: ID now
        [1]: Node_Pose (global)
        [2]: Landmark
        [3]: Scan
        -------------------------
        Edge Set
        [0]: ID from
        [1]: ID to
        [2]: Edge Pose (relative)
        [3]: Covariance 
        '''
        Node_set = []
        Edge_set = []
        
        
        
        car = CAR('/solamr_1/scan_lidar')
        
        # Declaration of publisher
        node_pub = rospy.Publisher("/solamr_1/collector_node", Node_List, queue_size=10)
        edge_pub = rospy.Publisher("/solamr_1/collector_edge", Edge_List, queue_size=10)
        
        # current_node = Node()
        # current_edge = Edge()

        Node_history = Node_List()
        Edge_history = Edge_List()

        while not rospy.is_shutdown():
            node_id = 0
            
            if car.lidar.landmark is not None: 
                if len(Node_set) > 0:
                    node_id = Node_set[-1][0] + 1

                # Store Node data
                Node_set.append([
                    node_id,
                    car.odom.x,
                    car.odom.y, 
                    car.odom.yaw,
                    car.lidar.landmark,
                    car.lidar.scan
                ])
                

                Node_ID_list = []
                global_pose_x_list = []
                global_pose_y_list = []
                global_pose_yaw_list = []
                current_scan_list = []
                
                for i in range(len(Node_set)):
                    Node_ID_list.append(Node_set[i][0])
                    global_pose_x_list.append(Node_set[i][1])
                    global_pose_y_list.append(Node_set[i][2])
                    global_pose_yaw_list.append(Node_set[i][3])
                    current_scan_point = Scan()
                    current_scan_point.point_x = Node_set[i][5][:,0]
                    current_scan_point.point_y = Node_set[i][5][:,1]
                    current_scan_list.append(current_scan_point)

                Node_history.Number_Node = len(Node_set)
                Node_history.Node_ID = Node_ID_list
                Node_history.global_pose_x = global_pose_x_list
                Node_history.global_pose_y = global_pose_y_list
                Node_history.global_pose_yaw = global_pose_yaw_list
                Node_history.current_scan = current_scan_list
                
                node_pub.publish(Node_history)


                # current_node.Node_ID = Node_set[-1][0]
                # current_node.global_pose.x = Node_set[-1][1]
                # current_node.global_pose.y = Node_set[-1][2]
                # current_node.global_pose.yaw = Node_set[-1][3]
                # # current_node.Landmark = Node_set[-1][4]
                # # current_node.Scan = Node_set[-1][5]
                # node_pub.publish(current_node)


                # Store Edge data with t = k & t = k - 1
                if len(Node_set) >= 2:
                    A = Node_set[-2]
                    B = Node_set[-1]

                    _, _, pose = ScanMatching().icp(A[5], B[5])

                    Cov = np.array(
                        [20,  0, 0,
                          0, 20, 0,
                          0,  0, 10000]
                    )
                    Edge_set.append([
                        Node_set[-2][0],
                        Node_set[-1][0],
                        pose,
                        Cov
                    ])


                    Node_ID_From_list = []
                    Node_ID_To_list = []
                    relative_pose_x_list = []
                    relative_pose_y_list = []
                    relative_pose_yaw_list = []
                    for i in range(len(Edge_set)):
                        Node_ID_From_list.append(Edge_set[i][0])
                        Node_ID_To_list.append(Edge_set[i][1])
                        relative_pose_x_list.append(Edge_set[i][2][0])
                        relative_pose_y_list.append(Edge_set[i][2][1])
                        relative_pose_yaw_list.append(Edge_set[i][2][2])



                    Edge_history.Number_Edge = len(Edge_set)
                    Edge_history.Node_ID_From = Node_ID_From_list
                    Edge_history.Node_ID_To = Node_ID_To_list
                    Edge_history.relative_pose_x = relative_pose_x_list
                    Edge_history.relative_pose_y = relative_pose_y_list
                    Edge_history.relative_pose_yaw = relative_pose_yaw_list
                    edge_pub.publish(Edge_history)




                    # current_edge.Node_ID_From = Edge_set[-1][0]
                    # current_edge.Node_ID_To = Edge_set[-1][1]
                    # current_edge.relative_pose.x = Edge_set[-1][2][0]
                    # current_edge.relative_pose.y = Edge_set[-1][2][1]
                    # current_edge.relative_pose.yaw = Edge_set[-1][2][2]
                    # current_edge.covariance = Edge_set[-1][3]
                    # current_edge.covariance_shape.row = 3
                    # current_edge.covariance_shape.column = 3
                    # edge_pub.publish(current_edge)

                if car.flag_getting_updated is True:
                    # update the node info here
                    print("Updated Done")
                    print(car.updated_node)
                    car.flag_getting_updated = False
                    pass

                if node_id > 300:
                    break
                
                
                rate.sleep()
            
        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        pass
        
