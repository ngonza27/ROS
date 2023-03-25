# ROS

## To test the turtle_move.py folllow these steps:

1. Create a new ROS package in your catkin_ws
2. Replace the src folder with the content of this repo
3. cd into your catkin_ws and run -> catkin_make to register the package
4. Source your workspace or your ~/.bashrc
5. Run in separate terminals -> roscore
                             -> rosrun turtlesim turtlesim_node
                             -> rosrun cave_gdp turtlebot_move.py
