#!/usr/bin/python
import rospy
from tf.transformations import quaternion_from_euler
import tf2_ros 
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
from tf_conversions import transformations as tf


class generator:
    def __init__(self):
        self.yaw_0 = rospy.get_param("yaw")
        self.x_0 = rospy.get_param("x")
        self.y_0 = rospy.get_param("y")

        self.set_initial_pose()

        self.pub = rospy.Publisher('/path', Path, queue_size=10)
        self.sub = rospy.Subscriber('/clicked_point', PointStamped, self.calc_path)
        # self.sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.calc_path)

    def calc_path(self, goal):   
        path = [[-3.5, 0.0],
                [-3.5, 3.5],
                [1.5, 3.5],
                [1.5, -1.5],
                [3.5, -1.5],
                [3.5, -8.0],
                [-2.5, -8.0],
                [-2.5, -5.5],
                [1.5, -5.5],
                [1.5, -3.5],
                [-1.0, -3.5]]

        while not rospy.is_shutdown():
            msg = Path()
            msg.header.frame_id = "odom"
            msg.header.stamp = rospy.Time.now()

            for wp in path:
                pose = PoseStamped()

                pose.pose.position.x = wp[0]*np.cos(self.yaw_0) + wp[1]*np.sin(self.yaw_0)
                pose.pose.position.y = - wp[0]*np.sin(self.yaw_0) + wp[1]*np.cos(self.yaw_0)

                msg.poses.append(pose)

            self.pub.publish(msg)

    
    def set_initial_pose(self):
        trans = TransformStamped()
        trans.header.stamp = rospy.Time.now()
        trans.header.frame_id = "map"
        trans.child_frame_id = "odom"        

        trans.transform.translation.x = self.x_0
        trans.transform.translation.y = self.y_0

        quat=tf.quaternion_from_euler(0.0,0.0,self.yaw_0)
        trans.transform.rotation.x = quat[0]
        trans.transform.rotation.y = quat[1]
        trans.transform.rotation.z = quat[2]
        trans.transform.rotation.w = quat[3]

        broadcaster=tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(trans)

if __name__ == '__main__':
    rospy.init_node('path_gen', anonymous=True)

    rospy.loginfo("Node init")

    generator()

    rospy.spin()