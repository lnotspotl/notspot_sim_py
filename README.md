# Notspot robot simulation
This repository contains all the files and code needed to simulate the notspot quadrupedal robot using [Gazebo](http://gazebosim.org/)  and [ROS](https://www.ros.org/).

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


This repository contains all the code and files needed to simulate the notspot quadrupedal robot using Gazebo and ROS.

This code has been tested on ROS noetic, so it might not work for other ROS versions.

If you happen to have problems with importing the Transformations module, make sure that the python folder RoboticsUtilities (notspot_sim/src/notspot_controller/scripts/RoboticsUtilities) is somewhere, from where python3 can import it. You can copy this folder together with all the files which it contains to this path: ~/.local/lib/python3.8/site-packages/

After you've downloaded this git repository, go to the src folder and init the workspace (catkin_init_workspace).
After that, simply got to the parent folder and run catkin_make. This compiles all the code and you're ready to start your simulation.

Run these commands:
source devel/setup.bash
roslaunch notspot run_robot_gazebo.launch

There might be a problem, where you cannot execute some of the python scripts. Go to the files, which contain these scripts and run the chmod +x command on the problematic scripts.

The robot is controlled via joystick and one has to experiment a bit to discover all the controllers. Make sure you play around the simulation a lot to discover all the capabilities of the robot.



This is my first github repo, I'm not that experienced with it thus far so I'm sorry for not being that much professional. I'll get better over time.

Credit:
https://github.com/mike4192/spotMicro
