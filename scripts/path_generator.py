#!/usr/bin/python
import rospy
import tf2_ros
import numpy as np
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped, Pose
from tf_conversions import transformations as tf
import cv2
import networkx as nx


class generator:
    def __init__(self):
        self.img_map = None
        self.thin = None
        self.Graph = nx.Graph()
        self.origin = None
        self.flag = False
        self.frame_id = ''
        self.res = 0.0

        # Listener base_footprint
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pub = rospy.Publisher('/plan', Path, queue_size=1)

        self.sub = rospy.Subscriber('/clicked_point', PointStamped, self.calc_path)
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.get_map)

    def calc_path(self, goal):
        while not self.flag:
            rospy.logwarn_once('Map not yet available')
            rospy.logwarn_once('Wait...')
        
        local_graph = self.Graph.copy()

        # Transformation from base_footprint to parent frame
        try:
            trans_fram_basef = self.tfBuffer.lookup_transform(
                self.frame_id, 'base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Could not find base_footprint transform')

        x_0, y_0 = [trans_fram_basef.transform.translation.x, trans_fram_basef.transform.translation.y]       
        px_0, py_0 = self.parentFrame2pixel([x_0, y_0])
        
        # Transformation from goal frame to parent frame
        try:
            trans_fram_goal = self.tfBuffer.lookup_transform(
                self.frame_id, goal.header.frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Could not find ' + goal.header.frame_id + ' transform')
                   
            
        mth = tf.translation_matrix([trans_fram_goal.transform.translation.x, trans_fram_goal.transform.translation.y, 0.0])
        q = [trans_fram_goal.transform.rotation.x, 
             trans_fram_goal.transform.rotation.y, 
             trans_fram_goal.transform.rotation.z,
             trans_fram_goal.transform.rotation.w]
        mth = np.matmul(mth, tf.quaternion_matrix(q))
        
        x_f, y_f = np.dot(mth, [goal.point.x, goal.point.y, 0.0, 1.0])[0:2]  
        px_f, py_f = self.parentFrame2pixel([x_f, y_f])
        
        local_graph.add_node('p_0', pos=(px_0, py_0))    
        local_graph.add_node('p_f', pos=(px_f, py_f))

        _, th = cv2.threshold(self.img_map, 245, 255, cv2.THRESH_BINARY)        
        ero = cv2.erode(th, None, iterations=8)
        
        no_black = cv2.countNonZero(ero)
        for i, p in local_graph.nodes.data('pos'):
            if i != 'p_0' and i != 'p_f':
                px, py = p
                p0_line = cv2.line(ero.copy(), (px_0, py_0), (px, py), 234, 1)
                pf_line = cv2.line(ero.copy(), (px_f, py_f), (px, py), 234, 1)

                if cv2.countNonZero(p0_line) == no_black:
                    x = (px_0 - p[0])
                    y = (py_0 - p[1])
                    local_graph.add_edge('p_0', i, weight=np.hypot(x, y))
                if cv2.countNonZero(pf_line) == no_black:
                    x = (px_f - p[0])
                    y = (py_f - p[1])
                    local_graph.add_edge('p_f', i, weight=np.hypot(x, y))

        try:
            vertexs = nx.shortest_path(local_graph, 'p_0', 'p_f')
        except Exception:
            rospy.logerr('No path between the robot position and clicked point')
            vertexs = []

        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = rospy.Time.now()

        for v in vertexs:
            pixel = np.array(local_graph.nodes[v]['pos'])
            position = self.pixel2parentFrame(pixel)
            pose = PoseStamped()
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]

            msg.poses.append(pose)

        self.pub.publish(msg)
        
    def get_thinning(self, image):
        _, th = cv2.threshold(image, 245, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        op = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)

        thin = cv2.ximgproc.thinning(op)

        return thin

    def get_map(self, grid_map):
        # Get map from data array
        self.flag = False
        self.origin = grid_map.info.origin
        self.frame_id = grid_map.header.frame_id
        self.res = grid_map.info.resolution
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

        img_map = np.uint8(np.array(img_map)/100.0 * -255.0 + 255.0)
        self.img_map = cv2.flip(img_map, 0)

        # Get thinned map
        self.thin = self.get_thinning(self.img_map)

        # Get graph
        corners = self.get_corners(self.thin)
        base_map = cv2.dilate(self.thin, None, iterations=11)        
        no_black = cv2.countNonZero(base_map)
        
        for i, p1 in enumerate(corners):
            self.Graph.add_node(i, pos=tuple(p1))
            for j, p2 in enumerate(corners):
                if i != j:
                    line_img = cv2.line(base_map.copy(), tuple(p1), tuple(p2), 255, 1)
                    
                    if cv2.countNonZero(line_img) == no_black:
                        x = (p1[0]-p2[0])
                        y = (p1[1]-p2[1])
                        self.Graph.add_edge(i, j, weight=np.hypot(x, y))
                        
        self.flag = True

    def get_corners(self, img):
        corner_regions = cv2.cornerHarris(img, 7, 7, 0.04)
        corner_regions = cv2.dilate(corner_regions, None)
        
        _, corner_regions = cv2.threshold(corner_regions, 0.001, 255, cv2.THRESH_BINARY)

        corner_regions = np.uint8(corner_regions)
        contours, _ = cv2.findContours(corner_regions, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        corners = []
        for i, c in enumerate(contours):
            M = cv2.moments(c)

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            corners.append((cX, cY))

        return corners
    
    def parentFrame2pixel(self, point):
        # Transformation from parent frame to img_map origin
        mth = tf.translation_matrix([-self.origin.position.x, -self.origin.position.y, 0.0])
        q = [self.origin.orientation.x, 
             self.origin.orientation.y, 
             self.origin.orientation.z,
             self.origin.orientation.w]
        yaw = tf.euler_from_quaternion(q)[2]
        mth = np.matmul(tf.euler_matrix(0.0, 0.0, -yaw),mth)
        
        pxl_left_bottom = np.intc(np.dot(mth, [point[0], point[1], 0.0, 1.0])/self.res)[0:2]
        
        return (pxl_left_bottom[0], self.img_map.shape[0] - pxl_left_bottom[1])
    
    def pixel2parentFrame(self, pixel):
        pxl_left_bottom = [pixel[0], self.img_map.shape[0] - pixel[1]]
        point_left_bottom = np.array(pxl_left_bottom)*self.res
        # Transformation from parent frame to img_map origin
        mth = tf.translation_matrix([self.origin.position.x, self.origin.position.y, 0.0])
        q = [self.origin.orientation.x, 
             self.origin.orientation.y, 
             self.origin.orientation.z,
             self.origin.orientation.w]
        yaw = tf.euler_from_quaternion(q)[2]
        mth = np.matmul(mth, tf.euler_matrix(0.0, 0.0, yaw))
        
        return tuple(np.dot(mth, [point_left_bottom[0], point_left_bottom[1], 0.0, 1.0])[0:2])


if __name__ == '__main__':
    rospy.init_node('path_gen', anonymous=True)

    rospy.loginfo("Node init")

    generator()

    rospy.spin()
