""" Code for fuzzy logic controller and follow behavior """

import fuzzylite as fl
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import numpy as np

class Follower:
    """" ROS node for follower robot controller """
    def __init__(self, robot_ns):
        rospy.init_node('Follower')

        self.vel_pub = rospy.Publisher(f'/{robot_ns}/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(f'/{robot_ns}/scan', LaserScan, self.process_scan)

    def process_scan(self, msg):
        """ Process lidar scan data and extracts distance measurements from left, right and front """
        pass

    def fuzzy_formation(self):
        """ Fuzzy logic controller that determines the robot commands to keep in formation """
        pass

    def fuzzy_collision_avoidance(self):
        """ Fuzzy logic controller that determines the robot commands to avoid obstacles """
        pass

    def fuzzy_fusion(self):
        """ Fuzzy logic controller that combines formation and collision avoidance """
        pass

    def get_heading(self):
        """ Determine heading of the follower in the world coordinate frame """
        pass

    def get_alpha(self):
        """ Calculate angle between follower and leader (alpha) """
        # TODO: Figure out how to use AR tags and computer vision to do this. If not possible, might
        # have to fall back onto having to use odometry to relay leader position (x, y) and 
        # calculate alpha to follower 
        pass

    def run(self):
        r = rospy.Rate(5)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

if __name__ == '__main__':
    node = Follower('robot2')
    node.run()