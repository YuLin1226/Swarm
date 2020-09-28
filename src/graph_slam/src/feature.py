#!/usr/bin/env python
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan


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
        
        # self.scan = np.zeros([])


        self.scan_feature = []
        rospy.Subscriber(topic_name, LaserScan, self.get_feature)
        



    def get_feature(self, msg):
        scan = msg.ranges
        f = Feature()
        
        xy_set = f._tf_polar_to_xy(scan)
        close_consecutive_dist_set, close_consecutive_dist_sum  = f._cal_close_consecutive_dist(xy_set)
        consecutive_dist_set,       consecutive_dist_sum        = f._cal_consecutive_dist(xy_set)
        consecutive_dist_std                                    = f._cal_consecutive_dist_standard_deviation(consecutive_dist_set)
        curvature_set                                           = f._cal_curvature(consecutive_dist_set)
        curvature_std                                           = f._cal_curvature_standard_deviation(curvature_set)
        x_mean,                     y_mean                      = f._cal_centroid(xy_set)
        geometry_std                                            = f._cal_geometry_standard_deviation(xy_set, x_mean, y_mean)
        range_std                                               = f._cal_range_standard_deviation(scan)

        self.scan_feature.append([
            close_consecutive_dist_sum,
            consecutive_dist_sum,
            consecutive_dist_std,
            curvature_std,
            [x_mean, y_mean],
            geometry_std,
            range_std
        ])

        print(self.scan_feature[-1])
  

if __name__ == "__main__":
    

    rospy.init_node('node_edge_construction', anonymous=True)

    try:
        car_feature = LiDAR_Association('/solamr_1/scan_lidar')
        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        pass