# [WIP] Port of pybullet_ros from ROS to ROS2

Following packages are required for succesfull compilation:
 
+  [xacro](https://github.com/ros/xacro)
+  [vision_opencv](https://github.com/ros2/vision_opencv)
+  [tf_transformations](https://github.com/DLu/tf_transformations/)

Currently only one launch is configured, which can be run with:

```
colcon build
source /ros2_foxy/install/local_setup.bash
source install/setup.bash
ros2 launch pybullet_ros2 bringup_robot_example_launch.py
```

Quicker for copy/pasting:

```
colcon build && source /ros2_foxy/install/local_setup.bash && source install/setup.bash && ros2 launch pybullet_ros2 bringup_robot_example_launch.py
```

Best way to run this is, is using `docker` and `vscode`. Just use [this development container](https://github.com/SimonDeussen/ros2-pybullet-development-container) and you should be ready to go in no time!