#!/usr/bin/python
import rospy
import tf2_ros 
import numpy as np
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
from tf_conversions import transformations as tf
import matplotlib.pyplot as plt
import cv2


class generator:
    def __init__(self):
        self.mapa = np.array([])
        self.skel = np.array([])
        
        self.pub = rospy.Publisher('/plan', Path, queue_size=10)
        
        self.sub = rospy.Subscriber('/clicked_point', PointStamped, self.calc_path)
        
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.get_map)
        self.pub_costmap = rospy.Publisher('/costmap', OccupancyGrid, queue_size=10)
        
        
        
        
        
    def calc_path(self, goal): 
        pose = PoseStamped()
        pose.pose.position.x = goal.point.x
        pose.pose.position.y = goal.point.y 

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.poses.append(pose)

        self.pub.publish(msg)

    def rotate_image(self, image, angle):
        image_center = tuple(np.array(image.shape[0:2]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1)

        vertical = cv2.warpAffine(image, rot_mat, image.shape[0:2], flags=cv2.INTER_CUBIC)
        im = vertical.copy()

        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                if i < 100 or j < 100 or j > 924 or i > 924:
                    im[i,j] = 205
                else:                    
                    neighbor = 0
                    if vertical[i+1,j] < 43.0:
                        neighbor += 1
                    if vertical[i-1,j] < 43.0:
                        neighbor += 1
                    if vertical[i+1,j-1] < 43.0:
                        neighbor += 1
                    if vertical[i+1,j+1] < 43.0:
                        neighbor += 1
                    if vertical[i-1,j+1] < 43.0:
                        neighbor += 1
                    if vertical[i-1,j-1] < 43.0:
                        neighbor += 1
                    if vertical[i,j+1] < 43.0:
                        neighbor += 1
                    if vertical[i,j-1] < 43.0:
                        neighbor += 1
                        
                    if neighbor >= 5:
                        im[i,j] = 0.0          
        return im
    
    def get_skeleton(self, image):
        _, th = cv2.threshold(image, 245, 255, cv2.THRESH_BINARY)        

        kernel = cv2.getStructuringElement(cv2.MORPH_OPEN, (3,3)) 
        op = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)

        skel = cv2.ximgproc.thinning(op)
    
        return skel
    
    def get_map(self, grid_map):
        mapa = []
        row = []
        
        for i in range(len(grid_map.data)):
            if grid_map.data[i] == -1:
                row.append(20)
            else:
                row.append(grid_map.data[i])
        
            if (i+1) % grid_map.info.width == 0:
                mapa.append(row)
                row = []
                
        mapa = np.array(mapa)/100.0*-255.0+255.0
        self.mapa = mapa.astype(np.uint8)
        
        rotated = self.mapa.copy()#rotate_image(self.mapa, -7.66)
        self.skel = self.get_skeleton(rotated)
        
        alpha = 0.3
        costmap = self.skel.copy()
        for i in range(1,4):
            blur = cv2.GaussianBlur(costmap,(10*i-1,10*i-1),20)
            _, th = cv2.threshold(blur, 2, 255, cv2.THRESH_BINARY)        
            costmap = cv2.addWeighted(th, alpha, costmap, 1-alpha, 0.0)
        
        alpha = 0.6
        _, th = cv2.threshold(rotated, 245, 255, cv2.THRESH_BINARY)
        
        costmap = cv2.addWeighted(th, 1-alpha, costmap, alpha, 0.0)
        costmap = costmap /255.0*100.0
        
        cm = OccupancyGrid()
        cm.header = grid_map.header       
        cm.info = grid_map.info
        
        cm.data = costmap.flatten().astype(np.uint8)
        
        
        self.pub_costmap.publish(cm)


if __name__ == '__main__':
    rospy.init_node('path_gen', anonymous=True)

    rospy.loginfo("Node init")

    generator()

    rospy.spin()
