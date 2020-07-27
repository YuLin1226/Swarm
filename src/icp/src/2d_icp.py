#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt


class ICP_2D():
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
            VT[2,:] *= -1
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

    def icp(self, A, B, init_pose=None, max_iterations=200, tolerance=0.000001):
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

        # make points homogeneous, copy them so as to maintain the originals
        src = np.ones((3, A.shape[0]))
        dst = np.ones((3, B.shape[0]))
        src[0:2,:] = np.copy(A.T)
        dst[0:2,:] = np.copy(B.T)
        
        # apply the initial pose estimation
        if init_pose is not None:
            src = np.dot(init_pose, src)

        prev_error = 0

        for i in range(max_iterations):
            # find the nearest neighbours between the current source and destination points
            distances, indices = self.nearest_neighbor(src[0:2,:].T, dst[0:2,:].T)

            # compute the transformation between the current source and nearest destination points
            T,_,_ = self.best_fit_transform(src[0:2,:].T, dst[0:2,indices].T)

            # update the current source
            # refer to "Introduction to Robotics" Chapter2 P28. Spatial description and transformations
            src = np.dot(T, src)

            # check error
            mean_error = np.sum(distances) / distances.size
            if abs(prev_error-mean_error) < tolerance:
                break
            prev_error = mean_error

        # calculcate final tranformation
        T,_,_ = self.best_fit_transform(A, src[0:2,:].T)

        return T, distances
    
    def plot_data(self, data, color_info, legend_info):
        '''
        data : Nx2 array
        '''
        x = data[:, 0]
        y = data[:, 1]
        
        plt.scatter(x, y, color=color_info, label=legend_info)
        

if __name__ == "__main__":
    # A = np.random.randint(0,51,(20,2))    # (3D) 20 points for test
    A = np.array([
        range(20),
        range(10)+[10]*10
    ]).T
    
    
    rotz = lambda theta: np.array([ [np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta) ]])
    trans = np.array([2.0, -1.0])
    B = A.dot( rotz(np.pi/4).T ) + trans
    
    TF_icp = ICP_2D()

    T, distances = TF_icp.icp(A, B)
    # C = (A - np.mean(A, axis=0)).dot(T[0:2, 0:2]) + T[0:2, 2] + np.mean(A, axis=0)
    C = A.dot(T[0:2, 0:2].T) + T[0:2, 2]



    T_sol = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0]
    ])
    T_sol[0:2, 0:2] = rotz(np.pi/4).T
    T_sol[0:2, 2] = trans

    
    
    

    np.set_printoptions(precision=3,suppress=True)
    print("answer:")
    print(T)
    print("solution:")
    print(T_sol)

    # TF_icp.plot_data(A, "black")
    TF_icp.plot_data(B, "blue", "B")
    TF_icp.plot_data(C, "red", "A'")
    plt.legend()
    plt.show()

    