""" Code for fuzzy logic controller and follow behavior """

import fuzzylite as fl
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import numpy as np
from avoidance_engine import avoidance_engine

class Follower:
    """" ROS node for follower robot controller """
    def __init__(self, robot_ns):
        rospy.init_node('Follower')

        # Setup publishers and subscribers
        # self.vel_pub = rospy.Publisher(f'/{robot_ns}/cmd_vel', Twist, queue_size=10)
        # rospy.Subscriber(f'/{robot_ns}/scan', LaserScan, self.process_scan)
        self.vel_pub = rospy.Publisher(f'/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(f'/scan', LaserScan, self.process_scan)

        # Define instance variables
        self.laser_distances = None

        # Setup Fuzzy Logic Controller Inputs
        self.left_laser = avoidance_engine.input_variable('Left_Laser')
        self.right_laser = avoidance_engine.input_variable('Right_Laser')
        self.front_laser = avoidance_engine.input_variable('Front_Laser')

        # Setup Fuzzy Logic Controller Outputs
        self.velocity = avoidance_engine.output_variable('Velocity')
        self.rotation = avoidance_engine.output_variable('Rotation')

    def process_scan(self, msg):
        """ Process lidar scan data and extracts distance measurements from left, right and front """
        angles = [90, 270, 0] # left, right, and front lidar angles
        self.laser_distances = [msg.ranges[theta] for theta in angles]
        
    def fuzzy_formation(self):
        """ Fuzzy logic controller that determines the robot commands to keep in formation """
        # Inputs to the formation FLC are angle (phi) and distance (d)
        # TODO: use AR tags now instead of following paper
        # phi = self.get_heading() - self.get_alpha()
        # d = self.get_distance_to_leader()
        pass


    def fuzzy_collision_avoidance(self):
        """ Fuzzy logic controller that determines the robot commands to avoid obstacles """
        # Skip if no laser distance data
        if self.laser_distances is None:
            return None, None

        # Feed lidar scan distances as input to the fuzzy controller
        self.left_laser.value = self.laser_distances[0]
        self.right_laser.value = self.laser_distances[1]
        self.front_laser.value = self.laser_distances[2]

        # Perform fuzzy inference
        avoidance_engine.process()

        print('Velocity:', self.velocity.value)
        print('Rotation:', self.rotation.value)

        return self.velocity.value, self.rotation.value

    def fuzzy_fusion(self, v2, r2):
        """ Fuzzy logic controller that combines formation and collision avoidance """
        # TODO: this setup is temporary to test avoidance engine. Add inputs from formation controller too
        return v2, r2

    def get_leader_position(self):
        """ Estimates leader position using AR tags. Determines leader relative angle to follower's center axis"""
        pass

    def get_distance_to_leader(sef):
        """ Calculates distance from follower to leader """
        pass

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # Call fuzzy controllers
            self.fuzzy_formation()
            v2, r2 = self.fuzzy_collision_avoidance()

            # Merge fuzzy controller outputs
            if v2 is not None:
                print('vel:', v2, 'ang vel:', r2)
                v_final, r_final = self.fuzzy_fusion(v2, r2)
                
                m = Twist()
                m.linear.x = v_final
                m.angular.z =  r_final

                self.vel_pub.publish(m)

            r.sleep()

        print("Shutting down")

if __name__ == '__main__':
    node = Follower('robot2')
    node.run()