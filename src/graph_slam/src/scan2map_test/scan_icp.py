#!/usr/bin/env python
from matplotlib import pyplot as plt
import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
from sklearn.neighbors import NearestNeighbors # For ICP

class Viewer:
    """
    Use matplotlib to show costmap and plot cmd_vel predicted locations.
    """
    def __init__(self):
        self.fig, self.ax = plt.subplots(3,1)
        self.ax1 = self.ax[0]
        self.ax2 = self.ax[1]
        self.ax3 = self.ax[2]
        plt.ion()
        plt.show()

    def update(self, scan):
        """
        Update matplotlib ax.
        """
        self._plot_scan_map_high_resolution()
        self._plot_scan_map_low_resolution()
        self._plot_robot()
        self._plot_raw_data(scan)
        plt.pause(0.01)
        self.fig.canvas.draw()
        


    def _plot_scan_map_high_resolution(self):
        
        global MAP_1, SIZE
        self.ax1.cla() 
        self.ax1.imshow(X=MAP_1[::-1], cmap="gray", origin="lower")  # use img to show costmap
        self.ax1.grid(which='both', color='grey')
        MAP_1 = np.zeros((SIZE,SIZE))

    def _plot_scan_map_low_resolution(self):
        
        global MAP_2, SIZE
        self.ax2.cla() 
        self.ax2.imshow(X=MAP_2[::-1], cmap="gray", origin="lower")  # use img to show costmap
        self.ax2.grid(which='both', color='grey')
        MAP_2 = np.zeros((SIZE,SIZE))

    def _plot_raw_data(self, scan):

        global MAP_1_list, MAP_2_list, MAP_3_list 

        map_1 = np.array(MAP_1_list)
        map_2 = np.array(MAP_2_list)
        map_3 = np.array(MAP_3_list)
        self.ax3.cla() 
        self.ax3.scatter(map_1[:,0], map_1[:,1], c='black', s=2, label="scan_1")
        self.ax3.scatter(map_2[:,0], map_2[:,1], c='red', s=2, label="scan_2")
        self.ax3.scatter(scan[:,0], scan[:,1], c='blue', s=2, label="scan_2")
        self.ax3.scatter(map_3[:,0], map_3[:,1], c='green', s=2, label="scan_2")
        self.ax3.grid(which='both', color='grey')

        MAP_1_list, MAP_2_list = [],[]
    def _plot_robot(self):
        """
        Plot robot with orientation (assuming the location is at the center).
        """

        global SIZE
        center = int(SIZE/2)

        try:
            x = center
            y = center
            self.ax1.plot(x, y, marker='o', markersize=10, linestyle='None')
            self.ax2.plot(x, y, marker='o', markersize=10, linestyle='None')
        except Exception as err:
            rospy.loginfo(err)

    

class ICP():
    def __init__(self):
        pass


    def best_fit_transform(self, A, B):
        '''
        Calculates the least-squares best-fit transform between corresponding 2D points A->B
        Input:
        A: Nx2 numpy array of corresponding 3D points
        B: Nx2 numpy array of corresponding 3D points
        Returns:
        T: 3x3 homogeneous transformation matrix
        R: 2x2 rotation matrix
        t: 2x1 column vector
        '''
        
        # If true, continue ; if false, break.
        assert len(A) == len(B)

        # translate points to their centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B

        # rotation matrix
        W = np.dot(BB.T, AA)
        U, s, VT = np.linalg.svd(W)
        R = np.dot(U, VT)

        # special reflection case
        if np.linalg.det(R) < 0:
            VT[-1, :] *= -1
            R = np.dot(U, VT)


        # translation
        t = centroid_B.T - np.dot(R, centroid_A.T)

        # homogeneous transformation
        T = np.identity(3)
        T[0:2, 0:2] = R
        T[0:2, 2] = t

        return T, R, t

    def nearest_neighbor(self, src, dst):
        '''
        Find the nearest (Euclidean) neighbor in dst for each point in src
        Input:
            src: Nx2 array of points
            dst: Nx2 array of points
        Output:
            distances: Euclidean distances (errors) of the nearest neighbor
            indecies: dst indecies of the nearest neighbor
        '''

        # indecies = np.zeros(src.shape[0], dtype=np.int)
        # distances = np.zeros(src.shape[0])
        
        # # i: index ; s: element in src
        # for i, s in enumerate(src):
        #     min_dist = np.inf

        #     # j: index ; d: element in dst
        #     for j, d in enumerate(dst):
        #         dist = np.linalg.norm(s-d)
        #         if dist < min_dist:
        #             min_dist = dist
        #             indecies[i] = j
        #             distances[i] = dist   
        
        neigh = NearestNeighbors(n_neighbors=1)
        neigh.fit(dst)
        distances, indices = neigh.kneighbors(src, return_distance=True)
        return distances.ravel(), indices.ravel()

        # return distances, indecies

    def icp(self, A, B, init_pose=None, max_iterations=10, tolerance=0.01, max_dist=0.5):
        '''
        The Iterative Closest Point method
        Input:
            A: Nx2 numpy array of source 3D points
            B: Nx2 numpy array of destination 3D point
            init_pose: 3x3 homogeneous transformation
            max_iterations: exit algorithm after max_iterations
            tolerance: convergence criteria
        Output:
            T: final homogeneous transformation
            distances: Euclidean distances (errors) of the nearest neighbor
        '''

        # Check the dimenstion is the same
        assert A.shape[1] == B.shape[1]  

        # get dimensions, m should be 2
        m = A.shape[1]
        


        # make points homogeneous, copy them so as to maintain the originals
        src = np.ones((m+1, A.shape[0]))
        dst = np.ones((m+1, B.shape[0]))
        src[0:m, :] = np.copy(A.T)
        dst[0:m, :] = np.copy(B.T)
        
        # apply the initial pose estimation
        if init_pose is not None:
            src = np.dot(init_pose, src)

        prev_error = 0

        for iteration in range(max_iterations):
            # find the nearest neighbours between the current source and destination points
            distances, indicies = self.nearest_neighbor(src[0:m, :].T, dst[0:m, :].T)


            # Remove Outliers
            j = 0
            src_good = []
            src_indicies_good = []
            dst_indicies_good = []
            for i, distance in enumerate(distances):
                if distance <= max_dist*2.0/(iteration + 1.0):
                # if distance <= max_dist:
                    src_good.append([
                        src[0, i],
                        src[1, i]
                    ])
                    dst_indicies_good.append(indicies[i])
                    src_indicies_good.append(i)
                    
                    j += 1

            src_good = np.array(src_good).T
            dst_good = dst[0:m, dst_indicies_good] 
            
            # compute the transformation between the current source and nearest destination points
            # T,_,_ = self.best_fit_transform(src[0:m, :].T, dst[0:m ,indicies].T)
            T,_,_ = self.best_fit_transform(src_good.T, dst_good.T)

            # update the current source, current source will converge to destination.
            src = np.dot(T, src)

            # check error
            mean_error = np.sum(distances) / distances.size
            if abs(prev_error-mean_error) < tolerance:
                break
            prev_error = mean_error

        # calculcate final tranformation
        T,_,_ = self.best_fit_transform(A, src[0:m, :].T)

        return T, distances
  

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
        transformation_history.append(np.hstack((rot, np.array([[closest_translation_x], [closest_translation_y]]))))


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

    global MAP_1,  MAP_1_list, SIZE
    center = int(SIZE/2)
    resolution = int(2000 / SIZE)

    for ind, dist in enumerate(msg.ranges):
        if dist >= msg.range_min and dist <= msg.range_max:
            
            # MAP_1_list.append([
            #     ind,
            #     dist
            # ])

            x = dist * math.cos(ind * math.pi / 725)
            y = dist * math.sin(ind * math.pi / 725)

            px = int(100*x/resolution) + center
            py = int(100*y/resolution) + center
            MAP_1[px,py] = -1
            MAP_1_list.append([
                x,
                y
            ])
            

    # for row in range(MAP_1.shape[0]):
    #     for col in range(MAP_1.shape[1]):

    #         if MAP_1[row, col] == -1:
    #             MAP_1_list.append([
    #                 row, col
    #             ])


def cb_scan_2(msg):


    # global MAP_1, COST, RAD_TABLE_COS, RAD_TABLE_SIN
    # COST = 0
    # last_COST = COST

    # for yaw in range(11):
    #     del_yaw = 0.25 + (yaw - 10)*0.05
    #     del_yaw_deg = del_yaw / math.pi * 360

    #     for x in range(21):
    #         del_x = -0.5 + (x - 30)*0.05
            
    #         for y in range(21):
    #             del_y = 0.5 + (y - 30)*0.05
    
    #             for ind, dist in enumerate(msg.ranges):
    #                 if dist >= msg.range_min and dist <= msg.range_max:
                        
    #                     theta = int(ind*0.25+del_yaw_deg)
    #                     theta_ind = theta % 360


    #                     scan_x = dist * RAD_TABLE_COS[theta_ind]/1000 + del_x
    #                     scan_y = dist * RAD_TABLE_SIN[theta_ind]/1000 + del_y

    #                     px = int(100*scan_x/5) + 200
    #                     py = int(100*scan_y/5) + 200
                        
    #                     if MAP_1[px,py] == -1:
    #                         COST += 1
        
    #             if COST >= last_COST:
    #                 sol = [yaw, x, y]

    #             last_COST = COST
    #             COST = 0

    # print("Solution is:", sol)
    # MAP_1 = np.zeros((400,400))

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

            x = dist * math.cos(ind * math.pi / 725)
            y = dist * math.sin(ind * math.pi / 725)

            px = int(100*x/resolution) + center
            py = int(100*y/resolution) + center
            
            MAP_2[px,py] = -1
            MAP_2_list.append([
                x,
                y
            ])


            x = dist * math.cos(ind * math.pi / 725 +0.3) - 0.5
            y = dist * math.sin(ind * math.pi / 725 +0.3) - 0.5

            MAP_3_list.append([
                x,
                y
            ])

    # for row in range(MAP_2.shape[0]):
    #     for col in range(MAP_2.shape[1]):

    #         if MAP_2[row, col] == -1:
    #             MAP_2_list.append([
    #                 row, col
    #             ])

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
    rospy.Subscriber(name='/solamr_2/scan_lidar', data_class=LaserScan, callback=cb_scan_2)

    # -- matplotlib show 
    viewer = Viewer()
    rate = rospy.Rate(hz=2)
    

    while not rospy.is_shutdown():
        
        if MAP_1_list and MAP_2_list :
            
            A = np.array(MAP_1_list)
            B = np.array(MAP_2_list)

            # print(A.shape)
            # print(B.shape)

            # T,_ = ICP().icp(A, B)
            T, scan_aligned, sol = icp(A, B)
            # v = t2v(T)
            # rot = np.eye(2)
            # x = 0
            # y = 0
            # for i in range(len(T)):
            #     rot = T[i][0:2,0:2]*rot
            #     x += T[i][0,2]
            #     y += T[i][1,2]

            print(sol)
            print("========================")
            # MAP_1 = np.zeros((SIZE,SIZE))
            # MAP_2 = np.zeros((SIZE,SIZE))
            # MAP_1_list = []
            # MAP_2_list = []

        viewer.update(scan_aligned)
        rate.sleep()
