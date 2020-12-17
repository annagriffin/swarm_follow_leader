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
        self.all_detected = None
        self.desired_distance = .8 # Desired distance to keep between leader and follower

        # Setup Fuzzy Logic Controller Inputs
        self.angle = formation_engine.input_variable('Angle')
        self.distance = formation_engine.input_variable('Distance')
        self.left_laser = avoidance_engine.input_variable('Left_Laser')
        self.right_laser = avoidance_engine.input_variable('Right_Laser')
        self.front_laser = avoidance_engine.input_variable('Front_Laser')

        # Setup Fuzzy Logic Controller Outputs
        self.vel_formation = formation_engine.output_variable('Velocity')
        self.rot_formation = formation_engine.output_variable('Rotation')
        self.vel_avoidance = avoidance_engine.output_variable('Velocity')
        self.rot_avoidance = avoidance_engine.output_variable('Rotation')

    def process_scan(self, msg):
        """ Process lidar scan data and extracts distance measurements from left, right and front """
        self.all_lidar_data = msg.ranges[:360]

        # TODO: Make more robust my taking an average of points around the directions instead of one lidar data point
        angles = [90, 270, 0] # left, right, and front lidar angles
        self.laser_distances = [self.get_average_distance(theta, 2) for theta in angles]
        self.all_detected = [i for i, angle in enumerate(msg.ranges) if not (angle == float('inf'))]
        
    def process_leader_angle(self, msg):
        """ Process offset from leader angle determined from the AR tags """
        self.offset_angle = msg.data

    def get_average_distance(self, angle, n, distance=None):
        """ 
        Gets the average of the neighboring lidar distances for a more robust distance measurement 
        
        Instead of relying on a single lidar distance measurement, the average of the +/- n nearby 
        angles are calculated for an average distance. If a lidar distance is inf it is not included
        in the average calculation. 

        Args:
            angle: the original lidar angle
            n: number of +/- surrounding lidar angles to perform average calculation
            distance: [optional] array of lidar distances
        Returns:
            average: the average of the angle and +/- neighboring angles
        """
        if self.all_lidar_data is None and distance is None:
            return None
            
        avg, count = 0, 0
        for i in range(-n, n+1):
            neighbor = self.all_lidar_data[(angle + i) % 360]
            # l.append(neighbor)
            if neighbor != float('inf'):
                avg += neighbor
                count += 1
        
        return avg / count if count else float('inf')

    def fuzzy_formation(self):
        """ Fuzzy logic controller that determines the robot commands to keep in formation """
        if self.offset_angle is None or self.all_lidar_data is None:
            return None, None

        # Feed offset angle and offset distance as input to the fuzzy controller
        # can anna get distance using ar tag?
        self.angle.value = self.offset_angle
        print('Lidar distance at direct angle:', self.all_lidar_data[(360-int(self.offset_angle))%360])
        actual_offset_distance = self.get_average_distance((360-int(self.offset_angle)), 3)

        if actual_offset_distance is None:
            return None, None

        self.distance.value = self.desired_distance - actual_offset_distance
        print('Distance value:', self.distance.value, 'actual distance:', actual_offset_distance)

        # Perform fuzzy inference
        formation_engine.process()

        # print('Distance value:', self.distance.value, actual_offset_distance)
        # print(self.vel_formation.name)

        return self.vel_formation.value, self.rot_formation.value

    def fuzzy_collision_avoidance(self):
        """ Fuzzy logic controller that determines the robot commands to avoid obstacles """
        # Skip if no laser distance data
        if self.laser_distances is None:
            return None, None

        # Feed lidar scan distances as input to the fuzzy controller
        self.left_laser.value = self.laser_distances[0] if self.laser_distances[0] else float('inf')
        self.right_laser.value = self.laser_distances[1] if self.laser_distances[0] else float('inf')
        self.front_laser.value = self.laser_distances[2] if self.laser_distances[0] else float('inf')

        # Perform fuzzy inference
        avoidance_engine.process()

        return self.vel_avoidance.value, self.rot_avoidance.value

    def fuzzy_fusion(self, v1, r1, v2, r2):
        """ Fuzzy logic controller that combines formation and collision avoidance """
        # TODO: this setup is temporary to test avoidance engine. Add inputs from formation controller too
        return v1, r1

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # Call fuzzy controllers
            v1, r1 = self.fuzzy_formation()
            v2, r2 = self.fuzzy_collision_avoidance()

            m = Twist()

            # Merge fuzzy controller outputs
            if v1 is not None and v2 is not None:
                # print('vel1:', v1, 'rot1:', r1, 'vel2:', v2, 'rot2:', r2)
                v_final, r_final = self.fuzzy_fusion(v1, r1, v2, r2)
                
                m.linear.x = v_final
                # m.linear.x = 0
                m.angular.z =  r_final

                # Reset distance and angle variables so that old data doesn't persist
                self.all_lidar_data = None
                self.laser_distances = None
                self.offset_angle = None
            else:
                m.linear.x = 0
                m.angular.z =  0

            self.vel_pub.publish(m)
      
            # if self.all_lidar_data and self.offset_angle:
            #     print(self.all_lidar_data)
            #     print(self.all_detected)
            #     print("Lidar distance at 0 degrees:", self.all_lidar_data[0])
            #     print(f'Offset Lidar distance at {int(self.offset_angle)} degrees, or {(360-int(self.offset_angle))%360} lidar degrees:', self.all_lidar_data[(360-int(self.offset_angle))%360])
            # else:
            #     # Reset data, probably have to move it cause not working
            #     self.all_lidar_data = None
            #     self.laser_distances = None
            #     self.offset_angle = None

            r.sleep()

        print("Shutting down")

if __name__ == '__main__':
    node = Follower('robot1')
    node.run()