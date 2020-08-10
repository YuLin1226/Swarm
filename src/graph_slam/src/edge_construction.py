#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

# class ICP():

#     def __init__(self):
#         pass


#     def best_fit_transform(self, A, B):
#         '''
#         Calculates the least-squares best-fit transform between corresponding 2D points A->B
#         Input:
#         A: Nx2 numpy array of corresponding 3D points
#         B: Nx2 numpy array of corresponding 3D points
#         Returns:
#         T: 3x3 homogeneous transformation matrix
#         R: 2x2 rotation matrix
#         t: 2x1 column vector
#         '''
        
#         # If true, continue ; if false, break.
#         assert len(A) == len(B)

#         # translate points to their centroids
#         centroid_A = np.mean(A, axis=0)
#         centroid_B = np.mean(B, axis=0)
#         AA = A - centroid_A
#         BB = B - centroid_B

#         # rotation matrix
#         W = np.dot(BB.T, AA)
#         U, s, VT = np.linalg.svd(W)
#         R = np.dot(U, VT)

#         # special reflection case
#         if np.linalg.det(R) < 0:
#             VT[-1,:] *= -1
#             R = np.dot(U, VT)


#         # translation
#         t = centroid_B.T - np.dot(R, centroid_A.T)

#         # homogeneous transformation
#         T = np.identity(3)
#         T[0:2, 0:2] = R
#         T[0:2, 2] = t

#         return T, R, t

#     def nearest_neighbor(self, src, dst):
#         '''
#         Find the nearest (Euclidean) neighbor in dst for each point in src
#         Input:
#             src: Nx2 array of points
#             dst: Nx2 array of points
#         Output:
#             distances: Euclidean distances (errors) of the nearest neighbor
#             indecies: dst indecies of the nearest neighbor
#         '''

#         indecies = np.zeros(src.shape[0], dtype=np.int)
#         distances = np.zeros(src.shape[0])
        
#         # i: index ; s: element in src
#         for i, s in enumerate(src):
#             min_dist = np.inf

#             # j: index ; d: element in dst
#             for j, d in enumerate(dst):
#                 dist = np.linalg.norm(s-d)
#                 if dist < min_dist:
#                     min_dist = dist
#                     indecies[i] = j
#                     distances[i] = dist   
        
#         return distances, indecies

#     def icp(self, A, B, init_pose=None, max_iterations=5, tolerance=0.01):
#         '''
#         The Iterative Closest Point method
#         Input:
#             A: Nx2 numpy array of source 3D points
#             B: Nx2 numpy array of destination 3D point
#             init_pose: 3x3 homogeneous transformation
#             max_iterations: exit algorithm after max_iterations
#             tolerance: convergence criteria
#         Output:
#             T: final homogeneous transformation
#             distances: Euclidean distances (errors) of the nearest neighbor
#         '''

#         # make points homogeneous, copy them so as to maintain the originals
#         src = np.ones((3, A.shape[0]))
#         dst = np.ones((3, B.shape[0]))
#         src[0:2,:] = np.copy(A.T)
#         dst[0:2,:] = np.copy(B.T)
        
#         # apply the initial pose estimation
#         if init_pose is not None:
#             src = np.dot(init_pose, src)

#         prev_error = 0

#         for i in range(max_iterations):
#             # find the nearest neighbours between the current source and destination points
#             distances, indices = self.nearest_neighbor(src[0:2,:].T, dst[0:2,:].T)

#             # compute the transformation between the current source and nearest destination points
#             T,_,_ = self.best_fit_transform(src[0:2,:].T, dst[0:2,indices].T)

#             # update the current source
#             # refer to "Introduction to Robotics" Chapter2 P28. Spatial description and transformations
#             src = np.dot(T, src)

#             # check error
#             mean_error = np.sum(distances) / distances.size
#             if abs(prev_error-mean_error) < tolerance:
#                 break
#             prev_error = mean_error

#         # calculcate final tranformation
#         T,_,_ = self.best_fit_transform(A, src[0:2,:].T)

#         return T, distances
    
#     def plot_data(self, data, color_info, legend_info):
#         '''
#         data : Nx2 array
#         '''
#         x = data[:, 0]
#         y = data[:, 1]
        
#         plt.scatter(x, y, color=color_info, label=legend_info)
        

from sklearn.neighbors import NearestNeighbors

class ICP():
    def best_fit_transform(self, A, B):
        '''
        Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
        Input:
        A: Nxm numpy array of corresponding points
        B: Nxm numpy array of corresponding points
        Returns:
        T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
        R: mxm rotation matrix
        t: mx1 translation vector
        '''
        assert A.shape == B.shape

        # get number of dimensions
        m = A.shape[1]

        # translate points to their centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B

        # rotation matrix
        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        # special reflection case
        if np.linalg.det(R) < 0:
            Vt[m-1,:] *= -1
            R = np.dot(Vt.T, U.T)

        # translation
        t = centroid_B.T - np.dot(R,centroid_A.T)

        # homogeneous transformation
        T = np.identity(m+1)
        T[:m, :m] = R
        T[:m, m] = t

        return T, R, t


    def nearest_neighbor(self, src, dst):
        '''
        Find the nearest (Euclidean) neighbor in dst for each point in src
        Input:
            src: Nxm array of points
            dst: Nxm array of points
        Output:
            distances: Euclidean distances of the nearest neighbor
            indices: dst indices of the nearest neighbor
        '''

        #assert src.shape == dst.shape  # commented by wuch

        neigh = NearestNeighbors(n_neighbors=1)
        neigh.fit(dst)
        distances, indices = neigh.kneighbors(src, return_distance=True)
        return distances.ravel(), indices.ravel()


    def icp(self, A, B, init_pose=None, max_iterations=20, tolerance=0.001, max_dist=1.0):
        '''
        The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
        Input:
            A: Nxm numpy array of source mD points
            B: Nxm numpy array of destination mD point
            init_pose: (m+1)x(m+1) homogeneous transformation
            max_iterations: exit algorithm after max_iterations
            tolerance: convergence criteria
            max_dist: maximum distance allowed between matching points
        Output:
            A_matched: (N_matched)xm numpy array of source points that have matching points in B
            B_matched: (N_matched)xm numpy array of destination points that are matched by A. (A_matched[i] will match B_matched[i])
            T: final homogeneous transformation that maps A on to B
            distances: Euclidean distances (errors) of the nearest neighbor
            i: number of iterations to converge
        '''

        #assert A.shape == B.shape
        # assert A.shape[1] == B.shape[1]  # modified by wuch

        # get number of dimensions
        # m = A.shape[1]

        # make points homogeneous, copy them to maintain the originals
        src = np.ones((3,A.shape[0]))  # source
        dst = np.ones((3,B.shape[0]))  # destination
        src[0:2,:] = np.copy(A.T)
        dst[0:2,:] = np.copy(B.T)

        # apply the initial pose estimation
        if init_pose is not None:
            src = np.dot(init_pose, src)

        prev_error = 0

        iteration_count = 0

        # for iteration in range(7):
        for iteration in range(max_iterations):
            iteration_count += 1

            # find the nearest neighbors between the current source and destination points
            distances, dst_indices = self.nearest_neighbor(src[0:2,:].T, dst[0:2,:].T)

            # remove outliers: process the output of nearest_neighbor, only being used to feed into best_fit_transform (added by wuch)
            count = 0
            for distance in distances:
                if distance <= max_dist*2.0/(iteration + 1.0):
                # if distance <= max_dist:
                    count += 1

            src_good = np.zeros((3, count))  # good means outliers removed
            dst_indices_good = np.zeros(count, dtype=int)
            src_indices_good = np.zeros(count, dtype=int)

            j = 0
            for i, distance in enumerate(distances):
                if distance <= max_dist*2.0/(iteration + 1.0):
                # if distance <= max_dist:
                    src_good[0][j] = src[0][i]
                    src_good[1][j] = src[1][i]
                    dst_indices_good[j] = dst_indices[i]
                    src_indices_good[j] = i
                    j += 1

                #else:  # this is an outlier
                    #dst_indices[i] = -1


            dst_good = dst[0:2, dst_indices_good]  
            # --- end of remove outliers

            # compute the transformation between the current source and nearest destination points
            #T,_,_ = best_fit_transform(src[:m,:].T, dst[:m,dst_indices].T)
            T,_,_ = self.best_fit_transform(src_good.T, dst_good.T)  # modified by wuch

            # update the current source
            src = np.dot(T, src)

            # check error
            mean_error = np.mean(distances)
            if np.abs(prev_error - mean_error) < tolerance:  # ternimate if error does not change much
                break
            prev_error = mean_error

        # calculate final transformation
        T,_,_ = self.best_fit_transform(A, src[0:2, :].T)

        # Matched A and B (A_matched[i] will match B_matched[i])  (added by wuch)
        #A_matched = A[src_indices_good,:]
        #B_matched = B[dst_indices_good,:]
        
        # --- test only (easy to plot: icp point-to-point)
        # return src_indices_good, dst_indices_good, T, distances, iteration

        # --- real work
        for i,dist in enumerate(distances):
            if dist > 0.8:
                dst_indices[i] = -1  # this is an outlier

        # return dst_indices
        return T






class LiDAR_Association():

    def __init__(self, topic_name='/solamr_1/scan_lidar'):
        
        # self.vel_pub = rospy.Publisher('/solamr_1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        self.px = []
        self.py = []
        self.scan = np.zeros((2,2))
        rospy.Subscriber(topic_name, LaserScan, self.get_lidar)
        
    def get_lidar(self, msg):
        point = []
        for ind, dist in enumerate(msg.ranges):
            if dist >= msg.range_min and dist <= msg.range_max:
                
                x = dist * math.cos(ind * math.pi / 725)
                y = dist * math.sin(ind * math.pi / 725)
                point.append([
                    x,
                    y
                ])
        self.px = [i[0] for i in point]
        self.py = [i[1] for i in point]
        self.scan = np.array(point).T


def t2v(T):
    '''(Done)
    homogeneous transformation to vector
    '''
    v = np.zeros((3,1))
    v[0:2, 0] = T[0:2, 2]
    v[2, 0] = math.atan2(T[1,0], T[0,0])
    return v

if __name__ == "__main__":
    rospy.init_node('node_edge_construction', anonymous=True)
    rate = rospy.Rate(1)

    try:
        car1 = LiDAR_Association('/solamr_1/scan_lidar')
        # car2 = LiDAR_Association('/solamr_2/scan_lidar')
        scan_last = np.array([])
        

        while not rospy.is_shutdown():
            is_empty = (scan_last.size == 0)
            scan_current = car1.scan 
            # print(np.shape(scan_current))
            # print(np.shape(scan_last))
            if not is_empty:
                T = ICP().icp(scan_current, scan_last)
                pose = t2v(T)
                print("==============")
                print("x: %f"%pose[0])
                print("y: %f"%pose[1])
                print("yaw: %f"%pose[2])

            scan_last = scan_current
            rate.sleep()

        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        pass





    