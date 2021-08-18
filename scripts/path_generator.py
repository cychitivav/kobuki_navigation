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
        self.img_map = np.array([])
        self.thin = np.array([])
        
        self.pub = rospy.Publisher('/plan', Path, queue_size=10)
        
        self.sub = rospy.Subscriber('/clicked_point', PointStamped, self.calc_path)        
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.get_map)
        
        
    def get_corners(self, img):
        corner_regions = cv2.cornerHarris(img, 3, 3, 0.042)   
        corner_regions = cv2.dilate(corner_regions, None)
        _, corner_regions = cv2.threshold(corner_regions, 0.01, 255, cv2.THRESH_BINARY)
        
        corner_regions = np.uint8(corner_regions)        
        contours, _ = cv2.findContours(corner_regions,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        corners = []
        for i, c in enumerate(contours):
            M = cv2.moments(c)
            
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            corners.append((cX,cY))
            
        return corners
        
        
    def calc_path(self, goal):             
        base_map = cv2.dilate(self.thin,None,iterations=4)
        color_map = cv2.cvtColor(base_map,cv2.COLOR_GRAY2RGB)        
        
        corners = self.get_corners(self.thin)
                  
        no_black = cv2.countNonZero(base_map)
        
        G = nx.Graph()  
        for i, p1 in enumerate(corners):
            for j, p2 in enumerate(corners):
                if p1 != p2:                                   
                    line_img = cv2.line(colormap.copy(), p1, p2, (234,0,234), 1)
                    
                    if cv2.countNonZero(cv2.cvtColor(line_img,cv2.COLOR_BGR2GRAY)) == no_black: 
                        G.add_edge(i,j,weight=np.hypot(p1[0]-p2[0], p1[1]-p2[1]))
                    
        vertexs = nx.shortest_path(G,30,19) # Punto más optimo de inicio
        
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        
        pose = PoseStamped()        
        pose.pose.position.x = x_0
        pose.pose.position.y = y_0        
        msg.poses.append(pose)
        
        for v in vertex:
            pose.pose.position.x = corners[v,0] # Cambiar de coordenadas de imagen a coordenadas reales
            pose.pose.position.y = corners[v,1]            

            msg.poses.append(pose)

        pose.pose.position.x = goal.point.y
        pose.pose.position.y = goal.point.y
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
    
    def get_thinning(self, image):
        _, th = cv2.threshold(image, 245, 255, cv2.THRESH_BINARY)        

        kernel = cv2.getStructuringElement(cv2.MORPH_OPEN, (3,3)) 
        op = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)

        thin = cv2.ximgproc.thinning(op)
    
        return thin
    
    def get_map(self, grid_map):
        img_map = []
        row = []
        
        for i in range(len(grid_map.data)):
            if grid_map.data[i] == -1:
                row.append(20)
            else:
                row.append(grid_map.data[i])
        
            if (i+1) % grid_map.info.width == 0:
                img_map.append(row)
                row = []
                
        img_map = np.array(img_map)/100.0 * -255.0 + 255.0
        
        angle = list(rospy.get_param('origin'))[2]
        self.img_map = rotate_image(img_map, angle*180/np.pi)
        
        self.thin = self.get_thinnig(self.img_map)


if __name__ == '__main__':
    rospy.init_node('path_gen', anonymous=True)

    rospy.loginfo("Node init")

    generator()

    rospy.spin()
