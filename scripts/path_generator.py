#!/usr/bin/python
import rospy
import tf2_ros 
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
from tf_conversions import transformations as tf


class generator:
    def __init__(self):
        self.pub = rospy.Publisher('/plan', Path, queue_size=10)
        self.sub = rospy.Subscriber('/clicked_point', PointStamped, self.calc_path)

    def calc_path(self, goal):   
        pose = PoseStamped()
        pose.pose.position.x = goal.point.x
        pose.pose.position.y = goal.point.y 

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.poses.append(pose)

        self.pub.publish(msg)

    def voronoi(self, points, shape=(1024,1024)):
        depthmap = np.ones(shape,np.float)*1e308
        colormap = np.zeros(shape,np.int)

        def hypot(X,Y):
            return (X-x)**2 + (Y-y)**2

        for i,(x,y) in enumerate(points):
            paraboloid = np.fromfunction(hypot,shape)
            colormap = np.where(paraboloid < depthmap,i+1,colormap)
            depthmap = np.where(paraboloid < depthmap,paraboloid,depthmap)

        for (x,y) in points:
            colormap[x-1:x+2,y-1:y+2] = 0

        return colormap

if __name__ == '__main__':
    rospy.init_node('path_gen', anonymous=True)

    rospy.loginfo("Node init")

    generator()

    rospy.spin()