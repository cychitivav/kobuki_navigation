#!/usr/bin/python
import rospy
import tf2_ros 
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
from tf_conversions import transformations as tf
import matplotlib.pyplot as plt
import cv2


class generator:
    def __init__(self):
        self.pub = rospy.Publisher('/plan', Path, queue_size=10)
        self.sub = rospy.Subscriber('/clicked_point', PointStamped, self.calc_path)

    def calc_path(self, goal): 
        image = cv2.imread('map/map.pgm', 0)
        _, umbral = cv2.threshold(image, 245, 255, cv2.THRESH_BINARY)        

        kernel = cv2.getStructuringElement(cv2.MORPH_OPEN, (3,3)) 
        apertura = cv2.morphologyEx(umbral, cv2.MORPH_OPEN, kernel)

        skel = cv2.ximgproc.thinning(apertura)
        
        # cv2.imshow("Original", image)
        # cv2.imshow("Umbralizada", umbral)
        # cv2.imshow("Umbralizada filtrada", apertura)
        # cv2.imshow("Esqueleto", skel)
        # cv2.waitKey(0) 

        pose = PoseStamped()
        pose.pose.position.x = goal.point.x
        pose.pose.position.y = goal.point.y 

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.poses.append(pose)

        self.pub.publish(msg)

    def voronoi(self,points,shape=(500,500)):
        depthmap = np.ones(shape,np.float)*1e308
        colormap = np.zeros(shape,np.int)

        for i,(x,y) in enumerate(points):
            hypot = lambda (X,Y): (X-x)**2 + (Y-y)**2

            paraboloid = np.fromfunction(hypot,shape)
            colormap = np.where(paraboloid < depthmap,i+1,colormap)
            depthmap = np.where(paraboloid < depthmap,paraboloid,depthmap)

        for (x,y) in points:
            colormap[x-1:x+2,y-1:y+2] = 0.0

        return colormap

    def rotate_image(self)
        img = cv2.imread('maps/map.pgm')
        rotated = ndimage.rotate(img, -8) # 8 deg
        nimg = cv2.cvtColor(rotated, cv2.COLOR_RGB2GRAY)

        cv2.imwrite('map/rotated.pgm',nimg)

        return nimg

if __name__ == '__main__':
    rospy.init_node('path_gen', anonymous=True)

    rospy.loginfo("Node init")

    generator()

    rospy.spin()