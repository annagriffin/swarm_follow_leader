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

    def __init__(self, image_topic):
        """ Initialize the angle finder """
        rospy.init_node('angle_finder')
        self.cv_image = None                        # the latest image from the camera
        # used to convert ROS messages to OpenCV
        self.bridge = CvBridge()

        self.h_field_of_view = 1.3962634  # radians

        rospy.Subscriber(image_topic + 'image_raw', Image, self.process_image)
        rospy.Subscriber(image_topic + 'camera_info',
                         CameraInfo, self.process_camera_info)
        self.pub = rospy.Publisher('angle_to_leader', Float32, queue_size=10)
        self.tag_width = 0.4  # meters
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
        self.focal_length = self.get_focal_length(
            self.camera_width, self.h_field_of_view)


    def get_focal_length(self, width, fov):
        """
        Calculates focal length of camera

        Parameters
        ----------
        width : width of the camera view
        fov : field of view degrees in radians 

        Returns
        -------
        Field of view for the given camera view
        """

        return (width / 2) / (math.tan(fov/2))


    def get_distance_to_camera(self, corners):
        """
        Calculates distance of object to the camera
        Must already know width of tag and focal length of camera

        Parameters
        ----------
        corners: four points making up the detected tag

        Returns
        -------
        Distance in meters to the detected object
        """

        p_width = self.get_perceived_width(corners)

        return (self.tag_width * self.focal_length) / p_width


    def get_perceived_width(self, corners):
        """
        Calculates the width in pixels that the detected tag spans
        Finds the difference between the left most point and the right most point

        Parameters
        ----------
        corners: four points making up the detected tag

        Returns
        -------
        Width of detected object
        """

        x_vals = [i[0] for i in corners]

        return (max(x_vals) - min(x_vals))


    def get_angle(self, center_x, center_y):
        """
        Calculates angle of robot to tag from its center
        Center - 0 degrees
        Left of center - negative values
        Right of center - positive values

        Parameters
        ----------
        center_x : x position of detected tag
        center_y : y position of detected tag

        Returns
        -------
        Angle in degrees
        """

        return ((center_x - float(self.camera_width / 2)) / self.camera_width) * (math.degrees(self.h_field_of_view))


    def order_points(self, pts):
        """
        Rearrange points so that they are listed top-left, top-right, botton-right, bottom-left

        Parameters
        ----------
        pts : four points

        Returns
        -------
        Ordered coordinates
        """

        # initialize a list of coordinates 
        rect = np.zeros((4, 2), dtype="float32")

        # top-left point will have the smallest sum, whereas
        # bottom-right point will have the largest sum
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        # top-right point will have the smallest difference,
        # bottom-left will have the largest difference
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        return rect


    def four_point_transform(self, image, pts):
        """
        Transforms points from one image to new scene
        Reference points are identified and trnasformed to a new 

        Parameters
        ----------
        image : original image
        pts : four points from original image

        Returns
        -------
        Warped image
        """

        rect = self.order_points(pts)

        # new array with size of new image
        dst = np.array([
            [0, 0],
            [self.ref_dimension, 0],
            [self.ref_dimension, self.ref_dimension],
            [0, self.ref_dimension]], dtype="float32")

        # compute the perspective transform matrix and then apply it
        M = cv2.getPerspectiveTransform(rect, dst)

        warped = cv2.warpPerspective(
            image, M, (self.ref_dimension, self.ref_dimension))

        return warped


    def find_leader(self, detected_contours):
        """
        Identifies the contour of the closets marker and draws it
        Determines which contor is correct when there are multiple in view

        Parameters
        ----------
        detected_contours : list of contour candidates 
        """

        contour_of_interest = max(detected_contours, key=lambda x : cv2.contourArea(x))
        contour_poly_curve = cv2.approxPolyDP(contour_of_interest, 0.01 * cv2.arcLength(contour_of_interest, closed=True), closed=True)

        corners = np.reshape(
            (np.float32(contour_poly_curve)), (4, 2))

        # compute the center of the contour
        M = cv2.moments(contour_poly_curve)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        self.draw(contour_of_interest, contour_poly_curve, corners, cX, cY)

        if not self.focal_length is None:
            angle = self.get_angle(cX, cY)
            self.pub.publish(angle)

        warped = self.four_point_transform(
            self.cv_image, corners)

        cv2.imshow("warped", warped)


    def draw(self, contour, curve, corners, cX, cY):
        """
        Draw contour, corners, and center in window

        Parameters
        ----------
        contour : contour to draw
        curve : a contour poly curve 
        corners : four  points marking corners
        cX : x value of center point
        cY : y values of center point

        Returns
        -------
        Field of view for the given camera view
        """

        cv2.drawContours(
            self.cv_image, [curve], 0, (0, 0, 225), 1)

        # draw each corner
        for each in corners:
            cv2.circle(self.cv_image,
                        (each[0], each[1]), 4, (0, 0, 255), -1)

        # draw the contour and center of the shape on the image
        cv2.circle(self.cv_image, (cX, cY), 2, (255, 255, 255), -1)

        # label center point
        cv2.putText(self.cv_image, "center", (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


    def run(self):
        """ The main run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.cv_image is None:

                grey = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
                edged = cv2.Canny(grey, 30, 200)
                _, frame_thresh = cv2.threshold(edged, 220, 255, 0)
                contours, h = cv2.findContours(
                    frame_thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
                
                merged = [(contours[i], h[0][i]) for i in range(0,len(contours))]
                detected = []
                for item in merged:
                    contour = item[0]
                    heirachy = item[1]

                    contour_area = cv2.contourArea(contour)
                    contour_poly_curve = cv2.approxPolyDP(
                        contour, 0.01 * cv2.arcLength(contour, closed=True), closed=True)

                    # bounds for contour area size, farther away, the smaller the area
                    if 2000 < contour_area < 22600 and len(contour_poly_curve) == 4:
                        
                        # filters for holes
                        if(heirachy[0] == -1):
                            detected.append(contour)
                            
                # find the leader if multiple contours are detected
                if len(detected) > 0:
                    self.find_leader(detected)

                cv2.imshow('video_window', self.cv_image)
                cv2.waitKey(5)

            r.sleep()


if __name__ == '__main__':
    node = AngleFinder("/robot1/camera/")
    node.run()
