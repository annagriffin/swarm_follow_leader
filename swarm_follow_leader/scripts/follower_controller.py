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
        """ Process lidar scan data TODO: fill out with more accurate documentation """
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