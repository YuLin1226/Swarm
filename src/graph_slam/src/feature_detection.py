#!/usr/bin/env python
import rospy
import numpy as np
import math
import csv
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan

class Segment():
    def __init__(self):
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
            print("Segment Radius is %f"%r)
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
            x1 = seg[i][0]
            y1 = seg[i][1]
            # x1m = np.mean(x1)
            # y1m = np.mean(y1)
            if i+1 >= len(seg):
                break
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
            


        landmark =  [   ["corner",    corner_num],
                        ["circle",  circle_num] ]
        return landmark

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

if __name__ == "__main__":

    rospy.init_node('node_edge_construction', anonymous=True)

    try:
        car1 = LiDAR_Association('/solamr_1/scan_lidar')
        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        # print(np.shape(car1.scan))
        # print(car1.scan)

        x = car1.scan[:,0]
        y = car1.scan[:,1]
        # print(px)
        # x = car1.scan[0]
        # y = car1.scan[1]

        SEG = Segment()
        SEG.do_segment(x, y)
        SEG.seg = SEG.classify_segment_type(SEG.seg)
        SEG.seg = SEG.corner_merger_operation(SEG.seg)
        landmark = SEG.do_landmark(SEG.seg)
        # print(SEG.seg[0][2])
        # print(SEG.seg[1][2])
        # print(SEG.seg[2][2])
        print(landmark)

        plt.scatter(x, y, s=5, color="red")
        plt.show()
        pass