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
        self.img_map = np.array([])
        self.thin = np.array([])
        self.Graph = nx.Graph()
        self.origin = Pose()
        self.frame_id = ''

        # Listener base_footprint
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pub = rospy.Publisher('/plan', Path, queue_size=10)

        self.sub = rospy.Subscriber(
            '/clicked_point', PointStamped, self.calc_path)
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.get_map)

    def calc_path(self, goal):
        local_graph = self.Graph.copy()

        try:
            trans = self.tfBuffer.lookup_transform(
                self.frame_id, 'base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Could not find base_footprint transform')

        G.add_node('p_0', pos=(trans.transform.translation.x,
                   trans.transform.translation.y))

        try:
            trans = self.tfBuffer.lookup_transform(
                self.frame_id, goal.header.frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Could not find ' +
                         goal.header.frame_id + ' transform')
            
        mth = tf.translation_matrix([trans.transform.translation.x, trans.transform.translation.y, 0.0]) 
        mth = np.matmul(mth, tf.quaternion_matrix(trans.transform.rotation))
        
        xg, yg, _, _ = np.dot(mth, [goal.point.x, goal.point.y, 0.0])
            
        G.add_node('p_f', pos=(xg, yg))

        _, th = cv2.threshold(self.img_map, 245, 255, cv2.THRESH_BINARY)
        ero = cv2.erode(th, None, iterations=7)

        no_black = cv2.countNonZero(ero)

        for i, p in G.nodes.data('pos'):
            p0_line = cv2.line(ero, (x_0, y_0), p, 234, 1)
            pf_line = cv2.line(ero, (x_f, y_f), p, 234, 1)

            if cv2.countNonZero(p0_line) == no_black:
                x = x_0 - p[0]
                y = y_0 - p[1]
                local_graph.add_edge('p_0', i, weight=np.hypot(x, y))
            if cv2.countNonZero(pf_line) == no_black:
                x = x_f - p[0]
                y = y_f - p[1]
                local_graph.add_edge('p_f', i, weight=np.hypot(x, y))

        vertexs = nx.shortest_path(G, 'p_0', 'p_f')

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        for v in vertexs:
            position = np.array(local_graph.nodes[v]['pos'])
            pose = PoseStamped()
            pose.pose.position.x = position[0] + self.origin.position.x
            pose.pose.position.y = position[1] + self.origin.position.y

            msg.poses.append(pose)

        self.pub.publish(msg)

    def rotate_image(self, image, angle):
        image_center = tuple(np.array(image.shape[0:2]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1)

        vertical = cv2.warpAffine(
            image, rot_mat, image.shape[0:2], flags=cv2.INTER_CUBIC)
        im = vertical.copy()

        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                if i < 100 or j < 100 or j > 924 or i > 924:
                    im[i, j] = 205
                else:
                    neighbor = 0
                    if vertical[i+1, j] < 43.0:
                        neighbor += 1
                    if vertical[i-1, j] < 43.0:
                        neighbor += 1
                    if vertical[i+1, j-1] < 43.0:
                        neighbor += 1
                    if vertical[i+1, j+1] < 43.0:
                        neighbor += 1
                    if vertical[i-1, j+1] < 43.0:
                        neighbor += 1
                    if vertical[i-1, j-1] < 43.0:
                        neighbor += 1
                    if vertical[i, j+1] < 43.0:
                        neighbor += 1
                    if vertical[i, j-1] < 43.0:
                        neighbor += 1

                    if neighbor >= 5:
                        im[i, j] = 0.0
        return im

    def get_thinning(self, image):
        _, th = cv2.threshold(image, 245, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        op = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)

        thin = cv2.ximgproc.thinning(op)

        return thin

    def get_map(self, grid_map):
        # Get map from data array
        self.origin = grid_map.info.origin
        self.origin = grid_map.header.frame_id
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
        self.img_map = np.uint8(self.rotate_image(img_map, angle*180/np.pi))

        # Get thinned map
        self.thin = self.get_thinning(self.img_map)

        # Get graph
        base_map = cv2.dilate(self.thin, None, iterations=11)
        corners = self.get_corners(self.thin)
        no_black = cv2.countNonZero(base_map)

        res =  rospy.get_param('resolution')
        
        for i, p1 in enumerate(corners):
            self.Graph.add_node(i, pos=(p1[0]*res, p1[1]*res))
            for j, p2 in enumerate(corners):
                if i != j:
                    line_img = cv2.line(base_map.copy(), tuple(p1), tuple(p2), 205, 1)

                    if cv2.countNonZero(line_img) == no_black:
                        x = (p1[0]-p2[0]) * res
                        y = (p1[1]-p2[1]) * res
                        self.Graph.add_edge(i, j, weight=np.hypot(x, y))

    def get_corners(self, img):
        corner_regions = cv2.cornerHarris(img, 4, 3, 0.04)
        corner_regions = cv2.dilate(corner_regions, None)
        _, corner_regions = cv2.threshold(
            corner_regions, 0.001, 255, cv2.THRESH_BINARY)

        corner_regions = np.uint8(corner_regions)
        contours, _ = cv2.findContours(
            corner_regions, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        corners = []
        for i, c in enumerate(contours):
            M = cv2.moments(c)

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            corners.append((cX, cY))

        return corners


if __name__ == '__main__':
    rospy.init_node('path_gen', anonymous=True)

    rospy.loginfo("Node init")

    generator()

    rospy.spin()
