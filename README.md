# Notspot robot simulation
This repository contains all the files and code needed to simulate the notspot quadrupedal robot using [Gazebo](http://gazebosim.org/)  and [ROS](https://www.ros.org/).
The software runs on [ROS noetic](http://wiki.ros.org/noetic) and [Ubuntu 20.04](http://www.releases.ubuntu.com/20.04/). If you want to use a different ROS version, you might have to do some changes to the source code.

<img src="resources/notspot_render" width="400"> <img src="resources/notspot_render2" width="400"> 

## Setup
```
cd src && catkin_init_workspace
cd .. && catkin_make
source devel/setup.bash
roscd notspot_controller/scripts && chmod +x robot_controller_gazebo.py
cp -r RoboticsUtilities ~/.local/lib/python3.8/site-packages
roscd notspot_joystick/scripts && chmod +x ramped_joystick.py
```

## Run
```
source devel/setup.bash
roslaunch notspot run_robot_gazebo.launch
```
After all the nodes have started, you can start using your joystick to control the robot.


This is my first github repo, I'm not that experienced with it thus far so I'm sorry for not being that much professional. I'll get better over time.

## Credits:
 - mike4192: https://github.com/mike4192/spotMicro
