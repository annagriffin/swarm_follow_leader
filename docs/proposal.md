---
layout: page
title: Project Proposal
permalink: /proposal/
feature-img: "assets/img/sample_feature_img_3.png"
---

## What is the main idea of your project?
Swarm Follow the Leader


## Provide some motivation for your project. Why does your team want to pursue this? What possible applications does the project have? Given our discussions in class, are there possible implications for society?

Our team is interested in learning more about swarm robotics. Using openCV, we want to try to get robots to follow a leader and adjust according to the leader's movements. We switched from cooperative transport (you can read more about the pivot [here](https://annagriffin.github.io/swarm_follow_leader/2020/11/23/blog-post-1.html)) because we didn't think that it was a good fit in terms of scope and learning goals for this project. Following a target is a useful behavior in robotics because it introduces some level of autonomy to the system. 


## MVP and stretch goal?
**MVP:** A decentralized control scheme, 1 leader robot, 1 follower robot. Leader guides a robot swarm with the aim to keep a leader-follower formation without colliding with other swarm agents

**Possible Extensions:** Leader robot looks for follower robots. Upon discovery, presents its back AR tag to the Neato to start “syncing”. Other Neato now is a follower and follows the leader robot. Upon meeting another robot, the new robot goes to the back of the chain.

 
Preliminary Resources:<br>
[Fuzzy Logic](https://ieeexplore-ieee-org.olin.idm.oclc.org/document/8752696?arnumber=8752696)


## Timeline/Steps
1. Get AR tag Neato model working from roslaunch. Confirm that the ar tag is attached to the model
2. Place AR tag on front and back of neato model
3. Create leader movement behavior (could be pretty simplistic. Maybe moving in circles or predefined shapes or even a random path or decide if a teleop control makes more sense)
4. Program follower behavior of one robot following any back AR tag
5. Determine if follower is left of the AR tag, right of the AR tag or centered. 
6. Think about a more robust control logic? Seems like there are multiple methods. A lot use fuzzy logic.



## What do you view as the biggest risks to you being successful on this project?

We anticipate the robot simulation being the most risky part of this project. So far, we have only worked with one Neato in Gazebo. For this project, we will need to introduce multiple robots.
 
Richard has tried working with swarm in Gazebo with the neatos, but encountered challenges. He hopes that given the other frameworks and documentation that Melinda has provided will help ease this process.



## Personal Goals and Project Alignment
**Richard:**
The project aligns well with my goals. As long as we finish the project and spread out the workload across the duration of the project, I can see myself succeeding. This is because by just completing the project, my goal of getting an understanding of multiple robot communication would be met, and if we are able to do this in a well structured manner, not saving all the work to the last minute, my other goals will be met.
 
**Anna:** 
The project and my personal goes are well aligned. It is a reasonably scoped project and we have a timeline of different checkpoints we want to hit from now to the end of the semester. This will hopefully keep us on track and help me achieve my goal of managing my time well. I also feel confident that all progress made during this project will increase my understanding of mulit-robot systems since that is the main focus of our project.
