#!/usr/bin/env python3

""" This is a script that walks through some of the basics of working with
    images with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image, CameraInfo
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3


class AngleFinder(object):
    """ The AngleFinder is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a leader robot within.
        The node will calculate the angle between the current robot and a leader
        robot if one is in view. """

    def __init__(self, robot_ns):
        """ Initialize the angle finder """
        rospy.init_node('angle_finder')
        self.cv_image = None                        # the latest image from the camera
        # used to convert ROS messages to OpenCV
        self.bridge = CvBridge()

        self.h_field_of_view = 1.3962634  # radians

        rospy.Subscriber(f'/{robot_ns}/camera/image_raw', Image, self.process_image)
        rospy.Subscriber(f'/{robot_ns}/camera/camera_info',
                         CameraInfo, self.process_camera_info)
        self.pub = rospy.Publisher(f'/{robot_ns}/angle_to_leader', Float32, queue_size=10)
        # self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.K_matrix = None
        self.tag_width = 0.4 # meters
        self.camera_width = None
        self.focal_length = None
        self.ref_dimension = 400

        cv2.namedWindow('video_window')


    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.binary_image = cv2.inRange(
        self.cv_image, (200, 200, 200), (255, 255, 255))


    def process_camera_info(self, msg):
       
        self.camera_width = msg.width
        self.focal_length = self.get_focal_length(self.camera_width, self.h_field_of_view)


    def get_focal_length(self, width, fov):
        
        return (width / 2) / (math.tan(fov/2))

    
    def get_distance_to_camera(self, corners):

        p_width = self.get_perceived_width(corners)
        
        return (self.tag_width * self.focal_length) / p_width

    def get_perceived_width(self, corners):

        x_vals = [i[0] for i in corners]
        return (max(x_vals) - min(x_vals))


    def get_angle(self, center_x, center_y):


        return ((center_x - float(self.camera_width / 2)) / self.camera_width) * (math.degrees(self.h_field_of_view))



    def order_points(self, pts):
        # initialzie a list of coordinates that will be ordered
        # such that the first entry in the list is the top-left,
        # the second entry is the top-right, the third is the
        # bottom-right, and the fourth is the bottom-left
        rect = np.zeros((4, 2), dtype="float32")
        # the top-left point will have the smallest sum, whereas
        # the bottom-right point will have the largest sum
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        # now, compute the difference between the points, the
        # top-right point will have the smallest difference,
        # whereas the bottom-left will have the largest difference
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        # return the ordered coordinates
        return rect

    def four_point_transform(self, image, pts):
        # obtain a consistent order of the points and unpack them
        # individually
        rect = self.order_points(pts)

        dst = np.array([
            [0, 0],
            [self.ref_dimension, 0],
            [self.ref_dimension, self.ref_dimension],
            [0, self.ref_dimension]], dtype="float32")
        # compute the perspective transform matrix and then apply it
        M = cv2.getPerspectiveTransform(rect, dst)

        warped = cv2.warpPerspective(
        image, M, (self.ref_dimension, self.ref_dimension))
        # return the warped image
        return warped

    def run(self):
        """ The main run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.cv_image is None:

                grey = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
                edged = cv2.Canny(grey, 30, 200)
                _, frame_thresh = cv2.threshold(edged, 220, 255, 0)
                contours, _ = cv2.findContours(
                    frame_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    contour_area = cv2.contourArea(contour)
                    contour_poly_curve = cv2.approxPolyDP(
                        contour, 0.01 * cv2.arcLength(contour, closed=True), closed=True)
                    if 1000 < contour_area < 22600 and len(contour_poly_curve) == 4:
                        # Draw the selected Contour matching the criteria fixed
                        cv2.drawContours(
                            self.cv_image, [contour_poly_curve], 0, (0, 0, 225), 1)

                        corners = np.reshape(
                            (np.float32(contour_poly_curve)), (4, 2))

                        # d = self.get_distance_to_camera(corners)
                        # width = self.get_perceived_width(corners)

                        for each in corners:
                            cv2.circle(self.cv_image,
                                       (each[0], each[1]), 4, (0, 0, 255), -1)

                        # compute the center of the contour
                        M = cv2.moments(contour_poly_curve)
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        
                        # draw the contour and center of the shape on the image
                        cv2.circle(self.cv_image, (cX, cY), 2, (255, 255, 255), -1)
                      

                        cv2.putText(self.cv_image, "center", (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                        if not self.focal_length is None:
                            angle = self.get_angle(cX, cY)
                            self.pub.publish(angle)
                            print(angle)

                        warped = self.four_point_transform(
                            self.cv_image, corners)

                        # cv2.imshow("warped", warped)

                cv2.imshow('video_window', self.cv_image)
                cv2.waitKey(5)

            r.sleep()


if __name__ == '__main__':
    node = AngleFinder("robot1")
    node.run()
