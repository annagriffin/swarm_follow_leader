#!/usr/bin/env python3

""" Code for fuzzy logic controller and follow behavior """

import fuzzylite as fl
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
from avoidance_engine import avoidance_engine
from formation_engine import formation_engine

class Follower:
    """" ROS node for follower robot controller """
    def __init__(self, robot_ns):
        rospy.init_node('Follower')

        # Setup publishers and subscribers
        self.vel_pub = rospy.Publisher(f'/{robot_ns}/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(f'/{robot_ns}/scan', LaserScan, self.process_scan)
        rospy.Subscriber(f'/{robot_ns}/angle_to_leader', Float32, self.process_leader_angle)
        # self.vel_pub = rospy.Publisher(f'/cmd_vel', Twist, queue_size=10)
        # rospy.Subscriber(f'/scan', LaserScan, self.process_scan)

        # Define instance variables
        self.all_lidar_data = None
        self.laser_distances = None
        self.offset_angle = None
        self.desired_distance = .75 # Desired distance to keep between leader and follower

        # Setup Fuzzy Logic Controller Inputs
        # self.angle = formation_engine.input_variable('Angle')
        # self.distance = formation_engine.input_variable('Distance')
        # self.left_laser = avoidance_engine.input_variable('Left_Laser')
        # self.right_laser = avoidance_engine.input_variable('Right_Laser')
        # self.front_laser = avoidance_engine.input_variable('Front_Laser')

        # # Setup Fuzzy Logic Controller Outputs
        # self.vel_formation = formation_engine.output_variable('Velocity')
        # self.rot_formation = formation_engine.output_variable('Rotation')
        # self.vel_avoidance = avoidance_engine.output_variable('Velocity')
        # self.rot_avoidance = avoidance_engine.output_variable('Rotation')

    def process_scan(self, msg):
        """ Process lidar scan data and extracts distance measurements from left, right and front """
        self.all_lidar_data = msg.ranges

        # TODO: Make more robust my taking an average of points around the directions instead of one lidar data point
        angles = [90, 270, 0] # left, right, and front lidar angles
        self.laser_distances = [msg.ranges[theta] for theta in angles]
        
    def process_leader_angle(self, msg):
        """ Process offset from leader angle determined from the AR tags """
        self.offset_angle = msg.data

    def fuzzy_formation(self):
        """ Fuzzy logic controller that determines the robot commands to keep in formation """
        if self.offset_angle is None or self.all_lidar_data is None:
            return None, None

        # Feed offset angle and offset distance as input to the fuzzy controller
        # can anna get distance using ar tag?
        self.angle.value = self.offset_angle
        self.distance.value = self.desired_distance - self.all_lidar_data[(360-int(self.offset_angle))%360]

        # Perform fuzzy inference
        formation_engine.process()

        return self.vel_formation.value, self.rot_formation.value

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

        return self.vel_avoidance.value, self.rot_avoidance.value

    def fuzzy_fusion(self, v1, r1, v2, r2):
        """ Fuzzy logic controller that combines formation and collision avoidance """
        # TODO: this setup is temporary to test avoidance engine. Add inputs from formation controller too
        return v2, r2

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # Call fuzzy controllers
            # v1, r1 = self.fuzzy_formation()
            # v2, r2 = self.fuzzy_collision_avoidance()

            # # Merge fuzzy controller outputs
            # if v1 is not None and v2 is not None:
            #     print('vel1:', v1, 'rot1:', r1, 'vel2:', v2, 'rot2:', r2)
            #     v_final, r_final = self.fuzzy_fusion(v1, r1, v2, r2)
                
            #     m = Twist()
            #     m.linear.x = v_final
            #     m.angular.z =  r_final

            #     self.vel_pub.publish(m)
      
            if self.all_lidar_data and self.offset_angle:
                print(self.all_lidar_data)
                print("Lidar distance at 0 degrees:", self.all_lidar_data[0])
                print(f'Offset Lidar distance at {int(self.offset_angle)} degrees:', self.all_lidar_data[(360-int(self.offset_angle))%360])
            else:
                # Reset data
                self.all_lidar_data = None
                self.laser_distances = None
                self.offset_angle = None

            r.sleep()

        print("Shutting down")

if __name__ == '__main__':
    node = Follower('robot1')
    node.run()