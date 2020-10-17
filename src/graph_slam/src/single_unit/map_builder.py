#!/usr/bin/env python
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from car_msg.msg import Node, Edge, Optimized_Node, Node_List, Edge_List
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from bresenham import bresenham
import tf

# -- CLASS
class DATA_COLLECTOR():
    
    def __init__(self, topic_SUB_NODE):

        self.node_set = []


        # Subscriber Declaration
        rospy.Subscriber(topic_SUB_NODE, Node_List, self._get_node)
        

    def _get_node(self, node_data):
        self.node_set = []
        for i in range(node_data.Number_Node):

            self.node_set.append([
                node_data.Node_ID[i],
                node_data.global_pose_x[i],
                node_data.global_pose_y[i],
                node_data.global_pose_yaw[i],
                np.array((node_data.current_scan[i].point_x, node_data.current_scan[i].point_y)).T                
            ])
        

class MAP_BUILDER():

    def __init__(self):

        self.map = np.zeros((200,200))
        self.map_origin = np.zeros((2))
        self.resolution = 5 # unit: cm

        self.LOG_odd_occupied = 0.9
        self.LOG_odd_free = 0.7

        self.extend_map_size = 100 #unit: cm

        self.robot_pixel_x = 0
        self.robot_pixel_y = 0
        
    def transform_to_global(self, dx, dy, dyaw, vector):
        
        """
        input: vector >> Nx2 nd array.
        output: result >> Nx2 nd array.
        """

        scan = np.vstack((vector.T , np.ones((1, len(vector)))))
        c = math.cos(dyaw)
        s = math.sin(dyaw)
        tf = np.array([
            [c, -s, dx],
            [s,  c, dy],
            [0,  0,  1]
        ])
        
        result = tf.dot(scan)[0:2,:].T
        return result



    def _update_map(self, node_info):
        """
        Input: node_info should include pose & scan data.
        """
        
        dx = node_info[1]
        dy = node_info[2]
        dyaw = node_info[3]
        
        scan_in_world = self.transform_to_global(dx, dy, dyaw, node_info[4])


        for i in range(len(scan_in_world)):
            scan_x = scan_in_world[i,0]
            scan_y = scan_in_world[i,1]

            pixel_x = int(100*(scan_x - self.map_origin[0])/self.resolution)
            pixel_y = int(100*(scan_y - self.map_origin[1])/self.resolution)
            
            while pixel_x < 0 or pixel_x >= self.map.shape[0] or pixel_y < 0 or pixel_y >= self.map.shape[1]:
                if pixel_x < 0:
                    
                    new_map = np.zeros((int(self.extend_map_size/self.resolution), self.map.shape[1]))
                    self.map = np.vstack((new_map, self.map))
                    pixel_x += int(self.extend_map_size/self.resolution)
                    self.map_origin[0] -= self.extend_map_size/100

                elif pixel_x >= self.map.shape[0]:
                    new_map = np.zeros((int(self.extend_map_size/self.resolution), self.map.shape[1]))
                    self.map = np.vstack((self.map, new_map))


                if pixel_y < 0:
                    new_map = np.zeros((self.map.shape[0], int(self.extend_map_size/self.resolution)))
                    self.map = np.hstack((new_map, self.map))
                    pixel_y += int(self.extend_map_size/self.resolution)
                    self.map_origin[1] -= self.extend_map_size/100

                elif pixel_y >= self.map.shape[1]:
                    new_map = np.zeros((self.map.shape[0], int(self.extend_map_size/self.resolution)))
                    self.map = np.hstack((self.map, new_map))
            
            self.map[pixel_x,pixel_y] = self.map[pixel_x,pixel_y] + self.LOG_odd_occupied
            # if MAP_1[px,py] > 1:
            #     MAP_1[px,py] = 1

            pixel_free = list(bresenham(int(100*(dx - self.map_origin[0])/self.resolution), int(100*(dy - self.map_origin[1])/self.resolution), pixel_x, pixel_y))
            for i in range(len(pixel_free)-2):
                self.map[pixel_free[i+1]] = self.map[pixel_free[i+1]] - self.LOG_odd_free
                # if MAP_1[px,py] < -1:
                #     MAP_1[px,py] = -1

        self.robot_pixel_y = self.map.shape[0] - int(100*(dx - self.map_origin[0])/5)
        self.robot_pixel_x = int(100*(dy - self.map_origin[1])/5)


class Viewer:
    """
    Use matplotlib to show costmap and plot cmd_vel predicted locations.
    """
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        plt.ion()
        plt.show()

    def update(self, occ_grid_map, robot_pixel_pose):
        """
        Update matplotlib ax.
        """
        self._plot_scan_map(occ_grid_map)
        self._plot_robot(robot_pixel_pose[0], robot_pixel_pose[1])
        self.fig.canvas.draw()
        plt.pause(0.01)

        

    def _plot_scan_map(self, occ_grid_map):
        
        self.ax.cla() 
        self.ax.imshow(X=occ_grid_map[::-1], cmap="gray")#, origin="lower")  # use img to show costmap
        self.ax.grid(which='both', color='grey')
        

    def _plot_robot(self, pixel_robot_x, pixel_robot_y):
        
        try:
            self.ax.plot(pixel_robot_x, pixel_robot_y, marker='o', markersize=5, linestyle='None')

        except Exception as err:
            rospy.loginfo(err)

  

# -------------- MAIN --------------
if __name__ == "__main__":
    rospy.init_node('MAP_BUILDING_NODE', anonymous=True)
    rate = rospy.Rate(0.2)

    try:
        
        lidar_info = DATA_COLLECTOR(topic_SUB_NODE = '/solamr_1/collector_node')
        indoor_map = MAP_BUILDER()
        viewer = Viewer()

        # robot_map_pub = rospy.Publisher("/solamr_1/occupancy_grid_map", OccupancyGrid, queue_size=10)
        # robot_map = OccupancyGrid()

        while not rospy.is_shutdown():
            
            if len(lidar_info.node_set) > 0:                
                for i in range(len(lidar_info.node_set)):
                    indoor_map._update_map(lidar_info.node_set[i])
                
                

            viewer.update(indoor_map.map, [indoor_map.robot_pixel_x, indoor_map.robot_pixel_y])



            # # Occupancy Grid Map Data Process Part:
            
            # if len(lidar_info.node_set) > 0:

            #     q = tf.transformations.quaternion_from_euler(0, 0, lidar_info.node_set[0][3])

            #     robot_map.data = np.reshape(indoor_map.map, indoor_map.map.shape[0]*indoor_map.map.shape[1], order='F')
            #     robot_map.info.resolution = indoor_map.resolution
            #     robot_map.info.width = indoor_map.map.shape[1]
            #     robot_map.info.height = indoor_map.map.shape[0]
            #     robot_map.info.origin.position.x = indoor_map.map_origin[0]
            #     robot_map.info.origin.position.y = indoor_map.map_origin[1]
            #     robot_map.info.origin.position.z = 0
            #     robot_map.info.origin.orientation.x = q[0]
            #     robot_map.info.origin.orientation.y = q[1]
            #     robot_map.info.origin.orientation.z = q[2]
            #     robot_map.info.origin.orientation.w = q[3]

            #     robot_map_pub.publish(robot_map)

            rate.sleep()


    except KeyboardInterrupt:
        pass

    finally:
        pass
        
        