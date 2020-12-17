---
layout: post
title: Following the Leader
# tags: [test, sample]
---

## Decentralized Follower Controller
When we set out to create the leader follower system, we knew we wanted to make the swarm control scheme decentralized. This way, each robot in the swarm can independently process its environment and react accordingly without need for a central arbiter or “brain”. With a decentralized system, robots can be cheaply added and removed when necessary, and scaling the swarm out to n robots is just as easy as adding the necessary robots. In the context of our leader, to add another robot to the swarm would be as simple as spinning up a new ROS node for an additional robot controller.
 
Regarding the system architecture of the decentralized follower, we decided to do some research into the leader follower space and landed upon [this paper](https://ieeexplore.ieee.org/document/4058766) that we ended up loosely following. While the paper describes a centralized system that relays the leader’s position (x, y) to each follower, we use the AR tag system to determine where the leader is relative to the follower. 
 
## Follower Controller
The follower controller is built on a two layer, three fuzzy logic controller system as shown in the diagram below. We decided to use a fuzzy logic controller as opposed to more traditional PID controllers due to the uncertainty in the leader follower problem. A robot in formation must dynamically handle the uncertainty in the environment such that it doesn’t collide with external objects and teammates while still maintaining to follow its intended leader. Fuzzy logic is well suited to adapt to this problem as it uses fuzzy non-crip (meaning non-boolean) values and a more human approach to computing where an accurate mathematical model of the system is not necessary. Instead, a linguistic approach is used to process uncertain data using designed rules. Linguistic sets and membership functions determine how input data is processed and then fed into an inference engine backed by a fuzzy rule base. This way by only utilizing the inputs from sensors, an effective controller can be designed.

![controller](../../../assets/img/controller_architecture.png)

## Formation Controller
The first of our three fuzzy controllers is the Formation Controller which determines the optimal translational velocity and angular velocity to allow the follower robot to follow the leader. The inputs to the controller are the leader offset angle (angular difference of the leader’s position from the center vertical) and distance (difference between a predefined desired distance to keep between leader and follower and the real distance that is measured from the lidar). When the offset angle is positive then the follower knows that the leader is to the right of the robot and turns right to adjust. The same process is used to adjust when the offset angle is negative, but instead the robot turns left. When the distance input (d = d_des - d_actual) is negative then the follower is too far away from the leader and speeds up. When the distance input is positive the follower is too close to the leader and slows down. When the offset angle or distance is more or less zero, then the follower stops turning or moving. This behavior is described in the rule base below.

![formation rules](../../../assets/img/formation_rules.png)

## Collision Avoidance
The Collision Avoidance Controller determines the optimal translational velocity and angular velocity to keep the follower from colliding into anything. The inputs to this controller is the distances detected from the lidar in areas to the left, right and in front of the robot. The Collision Avoidance Controller rules shown below give the robot the ability to turn to avoid obstacles, move straight ahead if there are no direct obstacles, and reverse if there seems to be a head on collision.

![avoidance rules](../../../assets/img/avoidance_rules.png)

## Fusion Controller
The higher level Command Fusion Controller takes a position measurement (the absolute value of the distance input from the formation controller, this describes how well the robot is in position within the formation) and minimum laser measurement (the closest detected obstacle, this describes how demanding collision avoidance is currently) to output weights for Formation Controller outputs and Collision Avoidance Controller outputs. Using the given input, the Command Fusion Controller determines how important each controller is given the robot’s sensor at the given time with a heavier preference to keeping the robot in formation unless an obstacle is very close to the robot and collision may be likely. The weights are used in a weighted sum of the outputs from the Formation Controller and Collision Avoidance Controller such that a final velocity and angular velocity is calculated. This equation is defined as follows:

v = v_f * W_f + v_c * W_c<br>
ω = ω_f * W_f + ω_C * W_c
 
where v and ω are the final velocity and angular velocity, v_f and ω_f are the velocity and angular velocity from the Formation Controller, v_c and ω_c are the velocity and angular velocity from the Collision Avoidance Controller, and W_f and W_c are the calculated.

![fusion rules](../../../assets/img/fusion_rules.png)