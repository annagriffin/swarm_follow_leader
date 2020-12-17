# Follow the Neato
*CompRobot Final Project*<br>
Richard Gao and Anna Griffin
<br>

## Introduction
Following a leader is common behavior in robots. It requires a level of autonomy from the followers and an ability to react to the movements of another object in their field of view. Over the course of the semester, we worked together to implement a version of this system ourselves.  There are two main challenges that come with implementing a follow the leader system. The first is figuring out how the followers can detect the leader robot. We have chosen computer vision as our detection method and relied on the OpenCV library. The second half of the problem is determining a control system for the follower to describe the following actions. After research on different implementation methods, we have decided that Fuzzy Logic was the best fit for our project. 

## Implementation

### Finding the Leader
#### AR Tags
The first part of our project involves detecting and tracking a leader robot using computer vision. We modified the neato models with an AR tag so that they could be easily detected. Each robot has the same tag at the moment. That was an intentional decision that we made after we began implementation. Having unique tags on each robot would be necessary further down the road, especially when more robots get added, but it was not needed at the moment we discovered. The tag sits above the neato's body so that it remains clear of the camera. 
[image of robot with ar tag]

#### OpenCV
For the computer vision aspect of this project, we used OpenCV and the CVBridge package which allows seamless conversation between ROS image messages and OpenCV images. We subscribed to the `{robot}/camera/raw_image` topic to visualize what the robot was “seeing”. When a robot with a tag enters the view, the robot will know it is a leader if it detects the tag properly. The process for identifying the tags we put on the robots mainly consisted of general filtering and contour detection. Since there are always going to be other objects in view, like the robot the tag is attached to, it was important that we find a robust way to select the correct contour that outlined the tag. We approached this in a few different ways. We set bounds for the min and max area within the contours and approximate the contour using `cv2.approxPolyDP()` so that we can tell if it has a relatively rectangular shape. 

At this point, it is possible that there are multiple tags in a few that are being detected, therefore we need to be able to determine which one is the correct robot to be following. When OpenCV finds the contours, it also returns a hierarchy array depending on the retrieval mode that sorts and connects the contours in a specified way. We used `cv2.RETR_CCOMP` which distinguishes boundaries and holes. There are only two levels here which was enough information for this scenario.

The border on which the tag is mounted and the AR tag itself would often both get detected. The border was placed so that if multiple tags were in view but overlapping, the tag could still be identified which didn’t work without a border because of perspective. With further filtering of the detected contours, we could separate the external boundaries and the holes but looking at the hierarchy array. Another situation that we had to account for was when multiple tags were detected at once. We found the contour with the greatest area and took that as the leader’s position for that specific robot. 

#### Publishing the Angle
An important piece of information that the controller needs is the offset angle of the detected robot. The controller uses this information to determine how the robot moves in order to stay in the follow formation. Whenever an angle is calculated, it gets published to `/{robot}/angle_to_leader` topic so that it is available to the controller. 

To obtain the angle from the image view, we used the width of the camera image, and the horizontal field of view. A nice thing about working in the simulator is that we didn’t have to do much camera calibration. The camera doesn’t distort the image, unless specified, according to the [documentation](ros gazebo camera documentation link).  The `/camera/camera_info` [topic](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) provides the details about the camera’s calibration. We could double check the distortion values by subscribing to this topic to make sure what we read about the camera in gazebo checked out. 


### Controller









### System Architecture
Within our system, there are two different roles the robot takes on, they are either a leader or a follower. The leaders are controlled though a teleop keyboard control. The followers have cameras and when they have detected a leader's tag, they used tracking to determine how to move in order to stay in pursuit. The main piece of information that the followers use is the angle between them and the detected leader. 



### Read on!
Visit our project [website!](https://annagriffin.github.io/swarm_follow_leader/)

