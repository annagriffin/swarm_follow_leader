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
from geometry_msgs.msg import Twist, Vector3


class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        # used to convert ROS messages to OpenCV
        self.bridge = CvBridge()

        self.h_field_of_view = 1.3962634  # radians

        rospy.Subscriber(image_topic, Image, self.process_image)
        rospy.Subscriber('/robot1/camera/camera_info',
                         CameraInfo, self.process_camera_info)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.K_matrix = None
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.ref_dimension = 400

        cv2.namedWindow('video_window')

    def process_camera_info(self, msg):
        # print(msg)
        # print()
        self.K_matrix = msg.K
        self.width = msg.width
        self.focal_length = self.get_focal_length(self.width, self.h_field_of_view)


    def get_focal_length(self, width, fov):
        
        return (width / 2) / (math.tan(fov/2))

    
    def get_distance_to_camera(self):

        return (width * focal_length) / p_width

    def get_perceived_width(self, corners):

        x_vals = [i[0] for i in corners]
        return (max(x_vals) - min(x_vals))


    def get_angle(self, center_x, center_y):


        return (center_x - (with / 2)) * (focal_length / 2)


    def get_dist(self, center_x, center_y):
        pass


    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.binary_image = cv2.inRange(
            self.cv_image, (200, 200, 200), (255, 255, 255))



    def get_h_matrices(self, poly_curve, x, y, orientation=0):
        """
        get H-matrix for detected orientation
        :param poly_curve: the approximate poly curve yielded by approxPolyDP
        :param x: no. of rows in the image frame
        :param y: no. of columns in the image frame
        :param orientation: orientation of the tag represented using integers 0 to 3
        :return: the homogeneous or inverse-homogeneous transform matrix
        """

        orientations = {'bottom_right': 0,
                        'bottom_left': 1, 'top_right': 2, 'top_left': 3}
        x_width = np.zeros(4, dtype=int)
        y_width = np.zeros(4, dtype=int)
        x_center = np.array([0, x, x, 0])
        y_center = np.array([0, 0, y, y])

        # Define width to perform homogeneous transforms
        if orientation == orientations['bottom_right']:
            x_width[0], y_width[0] = poly_curve[0][0][0], poly_curve[0][0][1]
            x_width[1], y_width[1] = poly_curve[1][0][0], poly_curve[1][0][1]
            x_width[2], y_width[2] = poly_curve[2][0][0], poly_curve[2][0][1]
            x_width[3], y_width[3] = poly_curve[3][0][0], poly_curve[3][0][1]
        elif orientation == orientations['bottom_left']:
            x_width[0], y_width[0] = poly_curve[1][0][0], poly_curve[1][0][1]
            x_width[1], y_width[1] = poly_curve[2][0][0], poly_curve[2][0][1]
            x_width[2], y_width[2] = poly_curve[3][0][0], poly_curve[3][0][1]
            x_width[3], y_width[3] = poly_curve[0][0][0], poly_curve[0][0][1]
        elif orientation == orientations['top_right']:
            x_width[0], y_width[0] = poly_curve[2][0][0], poly_curve[2][0][1]
            x_width[1], y_width[1] = poly_curve[3][0][0], poly_curve[3][0][1]
            x_width[2], y_width[2] = poly_curve[0][0][0], poly_curve[0][0][1]
            x_width[3], y_width[3] = poly_curve[1][0][0], poly_curve[1][0][1]
        elif orientation == orientations['top_left']:
            x_width[0], y_width[0] = poly_curve[3][0][0], poly_curve[3][0][1]
            x_width[1], y_width[1] = poly_curve[0][0][0], poly_curve[0][0][1]
            x_width[2], y_width[2] = poly_curve[1][0][0], poly_curve[1][0][1]
            x_width[3], y_width[3] = poly_curve[2][0][0], poly_curve[2][0][1]
        else:
            print('Incorrect Orientation!!')
            quit()

        # Evaluate the A matrix
        a_mat = [[x_width[0], y_width[0], 1, 0, 0, 0, -x_center[0] * x_width[0], -x_center[0] * y_width[0], -x_center[0]],
                 [0, 0, 0, x_width[0], y_width[0], 1, -y_center[0] *
                  x_width[0], -y_center[0] * y_width[0], -y_center[0]],
                 [x_width[1], y_width[1], 1, 0, 0, 0, -x_center[1] *
                  x_width[1], -x_center[1] * y_width[1], -x_center[1]],
                 [0, 0, 0, x_width[1], y_width[1], 1, -y_center[1] *
                  x_width[1], -y_center[1] * y_width[1], -y_center[1]],
                 [x_width[2], y_width[2], 1, 0, 0, 0, -x_center[2] *
                  x_width[2], -x_center[2] * y_width[2], -x_center[2]],
                 [0, 0, 0, x_width[2], y_width[2], 1, -y_center[2] *
                  x_width[2], -y_center[2] * y_width[2], -y_center[2]],
                 [x_width[3], y_width[3], 1, 0, 0, 0, -x_center[3] *
                  x_width[3], -x_center[3] * y_width[3], -x_center[3]],
                 [0, 0, 0, x_width[3], y_width[3], 1, -y_center[3] * x_width[3], -y_center[3] * y_width[3], -y_center[3]]]
        # Get inverse homogeneous transform using svd
        _, _, v_h = np.linalg.svd(a_mat, full_matrices=True)
        h_mat = np.array(v_h[8, :] / v_h[8, 8]).reshape((-1, 3))
        inv_h = np.linalg.inv(h_mat)
        # Return inverse homogeneous transform
        return h_mat, inv_h

    def get_tag_orientation(self, img_frame):
        """ get orientation from the image frame
        :param img_frame: image frame from the video
        :return: orientation of the tag
        """
        # Check get_H_matrix function in superimpose for orientation notation
        orientations = {0: 0, 1: 0, 2: 0, 3: 0}
        # Orientation: Bottom Right
        for i in range(250, 301):
            for j in range(250, 301):
                orientations[0] += img_frame[i, j]
        # Orientation: Bottom Left
        for i in range(250, 301):
            for j in range(100, 151):
                orientations[1] += img_frame[i, j]
        # Orientation: Top Right
        for i in range(100, 151):
            for j in range(250, 301):
                orientations[2] += img_frame[i, j]
        # Orientation: Top Left
        for i in range(100, 151):
            for j in range(100, 151):
                orientations[3] += img_frame[i, j]

        return max(orientations, key=orientations.get)

    def get_tag_id(self, img_frame, orientation):
        """
          :param img_frame: current frame of the video
          :param orientation: orientation of the tag
          :return: tag ID
        """
        tag_id = ''
        keys = []
        # Check get_H_matrix function in superimpose.py for orientation notation
        if orientation == 0:
            keys = [1, 0, 2, 3]
        elif orientation == 1:
            keys = [3, 1, 0, 2]
        elif orientation == 2:
            keys = [2, 3, 1, 0]
        elif orientation == 3:
            keys = [0, 2, 3, 1]
        structure = {0: [200, 250, 200, 250], 1: [150, 200, 200, 250], 2: [
            200, 250, 150, 200], 3: [150, 200, 150, 200]}

        total = 0
        for key in keys:
            for i in range(structure[key][0], structure[key][1]):
                for j in range(structure[key][2], structure[key][3]):
                    total += img_frame[i][j]

            if (total / 2500) > 220:
                tag_id += '1'
            else:
                tag_id += '0'
        return tag_id

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

        num, Rs, Ts, Ns = cv2.decomposeHomographyMat(M, self.K_matrix)

    #   print("num: ", num)
    #   print("Rs: ", Rs)
    #   print("Ts: ", Ts)
    #   print("Ns: ", Ns)
    #   print()

    #   num possible solutions will be returned.

    #     Rs contains a list of the rotation matrix.
    #     Ts contains a list of the translation vector.
    #     Ns contains a list of the normal vector of the plane.

        x = 500
        y = 500
        H = Rs[0]
        h1 = H[0][1]
        h2 = H[0][2]
        h0 = H[0][0]
        h3 = H[1][0]
        h4 = H[1][1]
        h5 = H[1][2]
        h6 = H[2][0]
        h7 = H[2][1]
        h8 = H[2][2]

        tx = (h0*x + h1*y + h2)
        ty = (h3*x + h4*y + h5)
        tz = (h6*x + h7*y + h8)

        px = int(tx/tz)
        py = int(ty/tz)

        warped = cv2.warpPerspective(
        image, M, (self.ref_dimension, self.ref_dimension))
        # return the warped image
        return warped, (px, py)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                # print(self.cv_image.shape)

                img_org = deepcopy(self.cv_image)
                grey = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
                edged = cv2.Canny(grey, 30, 200)
                _, frame_thresh = cv2.threshold(edged, 220, 255, 0)
                contours, _ = cv2.findContours(
                    frame_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    contour_area = cv2.contourArea(contour)
                    contour_poly_curve = cv2.approxPolyDP(
                        contour, 0.01 * cv2.arcLength(contour, closed=True), closed=True)
                    if 2000 < contour_area < 22600 and len(contour_poly_curve) == 4:
                        # Draw the selected Contour matching the criteria fixed
                        cv2.drawContours(
                            self.cv_image, [contour_poly_curve], 0, (0, 0, 225), 1)

                        corners = np.reshape(
                            (np.float32(contour_poly_curve)), (4, 2))

                        width = self.get_perceived_width(corners)
                        print(width)

                        for each in corners:
                            cv2.circle(self.cv_image,
                                       (each[0], each[1]), 4, (0, 0, 255), -1)

                        # compute the center of the contour
                        # M = cv2.moments(contour_poly_curve)
                        # cX = int(M["m10"] / M["m00"])
                        # cY = int(M["m01"] / M["m00"])
                        # # draw the contour and center of the shape on the image
                        # cv2.circle(self.cv_image, (cX, cY), 2, (255, 255, 255), -1)
                        # cv2.putText(self.cv_image, "center", (cX - 20, cY - 20),
                        #     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        # # show the image

                        warped, pts = self.four_point_transform(
                            self.cv_image, corners)
                        # print("points:   ", pts)

                        cv2.circle(self.cv_image, pts, 4, (0, 255, 255), -1)

                        cv2.imshow("warped", warped)

                cv2.imshow('video_window', self.cv_image)
                cv2.waitKey(5)

            # start out not issuing any motor commands
            r.sleep()


if __name__ == '__main__':
    node = BallTracker("/robot1/camera/image_raw")
    node.run()
