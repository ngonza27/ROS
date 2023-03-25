
# ROS

### To test the turtle_move.py folllow these steps:
1. Create a new ROS package in your `catkin_ws` folder\
	 ex: `$ catkin_create_pkg [pkg_name] rospy geometry_msgs turtlesim std_msgs`
2. Replace the `src` folder with the content of this repo
3. `cd` into your `catkin_ws` and run  `catkin_make` to register the new package
4. Source your workspace or your `~/.bashrc`
5. Run in separate terminals 
```console
  $ roscore
  $ rosrun turtlesim turtlesim_node
  $ rosrun cave_gdp turtlebot_move.py
  ```

