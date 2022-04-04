# HAND-GESTURE-CONTROLLED-ROBOT
The project is all about controlling the real world robot with the help of hand gestures. Using hand gesture robot can move forward , backward, it can rotate clockwise ,anticlockwise, it can accelarate and stop



***
## DEPENDENCIES/PAKAGES
1. Ubuntu OS
2. Python3 (Libraries-OpenCV,Numpy,math,rospy,time)
3. ROS(Robot operating system , version-noetic) [Instalation Guide](http://wiki.ros.org/ROS/Installation "ROS") 
4. Gazebo
5. Turtlebot3 pakage [To be cloned in src folder of catkin_ws](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation "Turtlebot3")



***
## ALGORITHM
* **Detection of Hand_Gesture using opencv library:** <br />
First of all we detect the hand gestures by seperating out the skin color from the input frame using HSV and the mask then noise removing from image by using some operation i.e. dialation and errosion,filters then we will draw convex-hull and using  convexity defects to classify different hand gestures. For more information you can refer this :[Hand-Gesture-Detection](https://github.com/Deepshikhar/Hand_Gesture)

* **ROS:** <br />
The turtlebot3 package is cloned. In the python file, we import all the required packages. Then we create a ros publisher to publish the twist message in the rostopic cmd_vel. A maximum limit is set for the turtlebot3 models linear and angular velocity.

* **Controlling the turtlebot3 using diffrent hand_gestures:** <br />
 After detecting the hand gestures we publish the message (velocity or rotaion) corresponding to particular hand gesture to turtlebot3 via topic called cmd_vel


***
## HERE WE GO TO RUN THE PROGRAM
> Copy all the files using [git clone]( https://github.com/ash-S26/HAND-GESTURE-CONTROLLED-ROBOT.git) in main workspace
``` 
$ git clone https://github.com/....
```

> Export TURTLEBOT3_MODEL
```
$ export TURTLEBOT3_MODEL=burger
```

> Launch turtlebot
```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch  
```
> Open python file in new terminal
```
$ cd/Hand_Gestur_Controlled_Robot/codes/
$ hand_gestur_controlled_robot.py
```

