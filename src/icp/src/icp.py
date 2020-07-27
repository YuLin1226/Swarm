#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt


class ICP():
    def __init__(self):
        pass


    def best_fit_transform(self, A, B):
        '''
        Calculates the least-squares best-fit transform between corresponding 3D points A->B
        Input:
        A: Nx3 numpy array of corresponding 3D points
        B: Nx3 numpy array of corresponding 3D points
        Returns:
        T: 4x4 homogeneous transformation matrix
        R: 3x3 rotation matrix
        t: 3x1 column vector
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
            VT[2,:] *= -1
            R = np.dot(U, VT)


        # translation
        t = centroid_B.T - np.dot(R,centroid_A.T)

        # homogeneous transformation
        T = np.identity(4)
        T[0:3, 0:3] = R
        T[0:3, 3] = t

        return T, R, t

    def nearest_neighbor(self, src, dst):
        '''
        Find the nearest (Euclidean) neighbor in dst for each point in src
        Input:
            src: Nx3 array of points
            dst: Nx3 array of points
        Output:
            distances: Euclidean distances (errors) of the nearest neighbor
            indecies: dst indecies of the nearest neighbor
        '''

        indecies = np.zeros(src.shape[0], dtype=np.int)
        distances = np.zeros(src.shape[0])
        
        # i: index ; s: element in src
        for i, s in enumerate(src):
            min_dist = np.inf

            # j: index ; d: element in dst
            for j, d in enumerate(dst):
                dist = np.linalg.norm(s-d)
                if dist < min_dist:
                    min_dist = dist
                    indecies[i] = j
                    distances[i] = dist   
        
        return distances, indecies

    def icp(self, A, B, init_pose=None, max_iterations=5000, tolerance=0.00000001):
        '''
        The Iterative Closest Point method
        Input:
            A: Nx3 numpy array of source 3D points
            B: Nx3 numpy array of destination 3D point
            init_pose: 4x4 homogeneous transformation
            max_iterations: exit algorithm after max_iterations
            tolerance: convergence criteria
        Output:
            T: final homogeneous transformation
            distances: Euclidean distances (errors) of the nearest neighbor
        '''

        # make points homogeneous, copy them so as to maintain the originals
        src = np.ones((4,A.shape[0]))
        dst = np.ones((4,B.shape[0]))
        src[0:3,:] = np.copy(A.T)
        dst[0:3,:] = np.copy(B.T)

        # apply the initial pose estimation
        if init_pose is not None:
            src = np.dot(init_pose, src)

        prev_error = 0

        for i in range(max_iterations):
            # find the nearest neighbours between the current source and destination points
            distances, indices = self.nearest_neighbor(src[0:3,:].T, dst[0:3,:].T)

            # compute the transformation between the current source and nearest destination points
            T,_,_ = self.best_fit_transform(src[0:3,:].T, dst[0:3,indices].T)

            # update the current source
            # refer to "Introduction to Robotics" Chapter2 P28. Spatial description and transformations
            src = np.dot(T, src)

            # check error
            mean_error = np.sum(distances) / distances.size
            if abs(prev_error-mean_error) < tolerance:
                break
            prev_error = mean_error

        # calculcate final tranformation
        T,_,_ = self.best_fit_transform(A, src[0:3,:].T)

        return T, distances
    
    def plot_data(self, data, color_info, legend_info):
        '''
        data : Nx3 array
        '''
        x = data[:,0]
        y = data[:,1]
        z = data[:,2]
        
        plt.scatter(x,y, color=color_info, label=legend_info)
        

if __name__ == "__main__":
    # A = np.random.randint(0,101,(20,3))    # (3D) 20 points for test
    
    A = np.array([
        range(20),
        range(10)+[10]*10,
        [1]*20
    ]).T
    rotz = lambda theta: np.array([[np.cos(theta),-np.sin(theta),0],
                                       [np.sin(theta),np.cos(theta),0],
                                       [0,0,1]])
    trans = np.array([2.12, -0.2, 0])
    B = A.dot(rotz(np.pi/4).T) + trans
    
    T, distances = ICP().icp(A, B)
    
    # To eliminate the most impossible ans 
    while (T[0,3]**2 + T[1,3]**2 + T[2,3]**2) > 10:
        T, distances = ICP().icp(A, B)

    C = A.dot(T[0:3, 0:3].T) + T[0:3, 3]

    # ICP().plot_data(A)
    ICP().plot_data(B, "blue", "B")
    ICP().plot_data(C, "red", "A'")
    plt.legend()
    plt.show()
    np.set_printoptions(precision=3,suppress=True)
    print T