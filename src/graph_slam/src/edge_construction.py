#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
import csv
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
                
                if (d_sum - d) > self.d_thres or L == (len(x) - 1) or d_conti > self.d_break_thres:
                    
                    self.seg.append([
                        x[0:L+1],
                        y[0:L+1],
                        []
                    ])
                    x = x[L+1 : len(x)]
                    y = y[L+1 : len(y)]

                    break

            if len(x) <= 2:
                self.seg[-1][0] = self.seg[-1][0] + x
                self.seg[-1][1] = self.seg[-1][1] + y
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
            # print("Segment Radius is %f"%r)
            if r > self.R_thres:
                seg[i][2] = "line"
                
            else:
                seg[i][2] = "circle"
        
        return seg
                
    def corner_merger_operation(self, seg):
        k = 0
        i = 0
        for i in range(len(seg) - 1):
            
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

            val = cal_acos([ x1[-1] - x1[0], y1[-1] - y1[0] ], [x2[-1] - x2[0], y2[-1] - y2[0]])

            if seg[i][2] == "circle":
            
                # circle , circle
                if seg[i][2] == seg[i+1][2]:
                    # 合併Operation
                    seg[i][0] = np.hstack(( seg[i][0] , seg[i+1][0] ))
                    seg[i][1] = np.hstack(( seg[i][1] , seg[i+1][1] ))
                    # seg[i][0] = seg[i][0] + seg[i+1][0]
                    # seg[i][1] = seg[i][1] + seg[i+1][1]
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
                        # seg[i][0] = seg[i][0] + seg[i+1][0]
                        # seg[i][1] = seg[i][1] + seg[i+1][1]
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
                        # seg[i][0] = seg[i][0] + seg[i+1][0]
                        # seg[i][1] = seg[i][1] + seg[i+1][1]
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
                        # seg[i][0] = seg[i][0] + seg[i+1][0]
                        # seg[i][1] = seg[i][1] + seg[i+1][1]
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


class LiDAR_Association():

    def __init__(self, topic_name='/solamr_1/scan_lidar'):
        
        # self.vel_pub = rospy.Publisher('/solamr_1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        # self.px = []
        # self.py = []
        # self.scan = np.zeros([])
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
        scan = np.array(point)
        
        px = scan[:,0]
        py = scan[:,1]
        Seg = Segment(px, py)
        self.landmark = Seg.landmark
        print(self.landmark)
        print("=============")
        print("")
        # print(np.array(point).shape)
 


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

    X = np.linalg.solve(A, B)
    xc = X[0]
    yc = X[1]
    R  = ( X[2] + xc**2 + yc**2 )**(0.5)
    return xc, yc, R

def cal_dist(xi, yi, xf, yf):
    
    dx = xi - xf
    dy = yi - yf
    
    return (dx**2 + dy**2)**(0.5)

def read_csv(csv_file_scan='case1.csv'):
    
    '''
    Read the scan data set from the csv file.
    '''
    with open(csv_file_scan) as scan:
        scan_csv = csv.reader(scan)
        scan_list = []
        # Data from csv file are "string", 
        # so they need to be change to float type by "list(map(float, row))" operation.
        for row in scan_csv:
            x = list(map(float, row))
            scan_list.append(x)
        scan_set = np.array(scan_list)
    
    return scan_set

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
        
        scan_list = []
        Edge = []
        while not rospy.is_shutdown():
            
            # scan_list.append(car1.scan)
            # scan_list.append(car2.scan)
            
            # if len(scan_list) >= 3:
                    
            #     A = scan_list[-1]
            #     B = scan_list[-2]
                
            #     T,_ = ICP().icp(A,B)
            #     pose = t2v(T)
            #     print(pose)

            # if len(scan_list) >= 10:
                    
            #     A = scan_list[-1]
            #     B = scan_list[-2]
                
            #     T,_ = ICP().icp(A,B)
            #     pose = t2v(T)
            #     print("x: %f"%pose[0])
            #     print("y: %f"%pose[1])
            #     print("yaw: %f"%pose[2])

            
            rate.sleep()

        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        pass





    