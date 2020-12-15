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

        # Setup publishers and subscribers
        self.vel_pub = rospy.Publisher(f'/{robot_ns}/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(f'/{robot_ns}/scan', LaserScan, self.process_scan)

        # Define instance variables
        self.laser_distances = [-1, -1, -1]

    def process_scan(self, msg):
        """ Process lidar scan data and extracts distance measurements from left, right and front """
        angles = [90, 270, 0] # left, right, and front lidar angles
        self.laser_distances = [msg.ranges[theta] for theta in angles]
        
    def fuzzy_formation(self):
        """ Fuzzy logic controller that determines the robot commands to keep in formation """
        # Inputs to the formation FLC are angle (phi) and distance (d)
        # TODO: use AR tags now instead of following paper
        phi = self.get_heading() - self.get_alpha()
        d = self.get_distance_to_leader()


    def fuzzy_collision_avoidance(self):
        """ Fuzzy logic controller that determines the robot commands to avoid obstacles """
    pass

        
    def fuzzy_fusion(self):
        """ Fuzzy logic controller that combines formation and collision avoidance """
        pass

    def get_leader_position(self):
        """ Estimates leader position using AR tags. Determines leader relative angle to follower's center axis"""
        pass

    def get_distance_to_leader(sef):
        """ Calculates distance from follower to leader """
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