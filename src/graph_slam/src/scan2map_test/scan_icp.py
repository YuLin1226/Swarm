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
        self.fig, self.ax = plt.subplots(2,1)
        self.ax1 = self.ax[0]
        self.ax2 = self.ax[1]
        plt.ion()
        plt.show()

    def update(self):
        """
        Update matplotlib ax.
        """
        self._plot_scan_map_high_resolution()
        self._plot_scan_map_low_resolution()
        self._plot_robot()
        plt.pause(0.01)
        self.fig.canvas.draw()
        


    def _plot_scan_map_high_resolution(self):
        
        global MAP_1
        self.ax1.cla() 
        self.ax1.imshow(X=MAP_1[::-1], cmap="gray", origin="lower")  # use img to show costmap
        self.ax1.grid(which='both', color='grey')
        MAP_1 = np.zeros((400,400))

    def _plot_scan_map_low_resolution(self):
        
        global MAP_2
        self.ax2.cla() 
        self.ax2.imshow(X=MAP_2[::-1], cmap="gray", origin="lower")  # use img to show costmap
        self.ax2.grid(which='both', color='grey')
        MAP_2 = np.zeros((400,400))


    def _plot_robot(self):
        """
        Plot robot with orientation (assuming the location is at the center).
        """
        try:
            x = 200
            y = 200
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
  

# --

def cb_scan_1(msg):

    global MAP_1,  MAP_1_list
    
    
    for ind, dist in enumerate(msg.ranges):
        if dist >= msg.range_min and dist <= msg.range_max:
            
            x = dist * math.cos(ind * math.pi / 725)
            y = dist * math.sin(ind * math.pi / 725)

            px = int(100*x/5) + 200
            py = int(100*y/5) + 200
            MAP_1[px,py] = -1

            

    for row in range(MAP_1.shape[0]):
        for col in range(MAP_1.shape[1]):

            if MAP_1[row, col] == -1:
                MAP_1_list.append([
                    row, col
                ])


def cb_scan_2(msg):

    global  MAP_2,  MAP_2_list
    # MAP_2 = np.zeros((400,400))
    # MAP_2_list = []
    for ind, dist in enumerate(msg.ranges):
        if dist >= msg.range_min and dist <= msg.range_max:
            
            x = dist * math.cos(ind * math.pi / 725)
            y = dist * math.sin(ind * math.pi / 725)

            px = int(100*x/5) + 200
            py = int(100*y/5) + 200
            
            MAP_2[px,py] = -1


    for row in range(MAP_2.shape[0]):
        for col in range(MAP_2.shape[1]):

            if MAP_2[row, col] == -1:
                MAP_2_list.append([
                    row, col
                ])

if __name__ == "__main__":

    # -- global vars
    MAP_1 = np.zeros((400,400))
    MAP_2 = np.zeros((400,400))
    MAP_1_list = []
    MAP_2_list = []
    # -- ros node and params
    rospy.init_node(name="viewer", anonymous=False)
    
    # -- ros function
    rospy.Subscriber(name='/solamr_1/scan_lidar', data_class=LaserScan, callback=cb_scan_1)
    rospy.Subscriber(name='/solamr_2/scan_lidar', data_class=LaserScan, callback=cb_scan_2)

    # -- matplotlib show 
    viewer = Viewer()
    rate = rospy.Rate(hz=2)
    

    while not rospy.is_shutdown():
        
        # if MAP_1_list and MAP_2_list :
            
        #     A = 0.01*np.array(MAP_1_list)
        #     B = 0.01*np.array(MAP_2_list)

        #     # print(A.shape)
        #     # print(B.shape)

        #     # T,_ = ICP().icp(A, B)
        #     # print(T)
        #     viewer.update()
        #     MAP_1 = np.zeros((400,400))
        #     MAP_2 = np.zeros((400,400))
        #     MAP_1_list = []
        #     MAP_2_list = []

        # viewer.update()
        rate.sleep()
