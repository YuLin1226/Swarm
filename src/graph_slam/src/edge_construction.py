#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors


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

    def icp(self, A, B, init_pose=None, max_iterations=30, tolerance=0.01, max_dist=1):
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
        





class LiDAR_Association():

    def __init__(self, topic_name='/solamr_1/scan_lidar'):
        
        # self.vel_pub = rospy.Publisher('/solamr_1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        # self.px = []
        # self.py = []
        self.scan = np.zeros([])
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
        # self.px = [i[0] for i in point]
        # self.py = [i[1] for i in point]
        self.scan = np.array(point)
        # print(np.array(point).shape)
        

def t2v(T):
    '''(Done)
    homogeneous transformation to vector
    '''
    v = np.zeros((3,1))
    v[0:2, 0] = T[0:2, 2]
    v[2, 0] = math.atan2(T[0,1], T[0,0])
    return v

if __name__ == "__main__":
    rospy.init_node('node_edge_construction', anonymous=True)
    rate = rospy.Rate(1)

    try:
        car1 = LiDAR_Association('/solamr_1/scan_lidar')
        # car2 = LiDAR_Association('/solamr_2/scan_lidar')
        scan_last = np.array([])
        print(len(scan_last))
        
        while not rospy.is_shutdown():
            is_empty_last = (len(scan_last) == 0)
            is_empty_current = (len(car1.scan) == 0)
            
            # scan_current = car1.scan 
            
            if not is_empty_current and not is_empty_last:
                print(car1.scan.shape, scan_last.shape)
                T,_ = ICP().icp(car1.scan, scan_last)
                print("======= ICP =======")
                pose = t2v(T)
                
                print("x: %f"%pose[0])
                print("y: %f"%pose[1])
                print("yaw: %f"%pose[2])
            # print("======= current =====")
            # print(car1.scan)
            # print("======= last ======")
            # print(scan_last)
            # print("hello")
            scan_last = car1.scan
            rate.sleep()

        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        pass





    