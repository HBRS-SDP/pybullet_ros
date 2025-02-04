# pybullet_ros

A bridge between [ROS1](https://www.ros.org/) and [PyBullet](https://pybullet.org/wordpress/)

<img src="https://github.com/oscar-lima/pybullet_ros/blob/noetic/common/images/r2d2_rviz.png" alt="drawing" width="600"/>

# Project status

This project is in a medium stage and presents with the following features:

- body velocity control - Subscription to cmd_vel topic and apply desired speed to the robot (without noise)
- joint control: Joint Position, velocity and effort control for all revolute joints on the robot
- sensors: Odometry, joint states (joint position, velocity and effort feedback), laser scanner, RGB camera image

Missing:

- [sdf](http://sdformat.org) support

Main implementation is done [here](https://github.com/oscar-lima/pybullet_ros/blob/noetic/ros/src/pybullet_ros/pybullet_ros.py)

## Installation

The following instructions have been tested under ubuntu 20.04 and [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).

This wrapper requires that you have pybullet installed, you can do so by executing:

        sudo -H pip3 install pybullet

Additionally clone this repository inside your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace),
compile (catkin build) and source your devel workspace (as you would normally do with any ROS pkg).

In case you need to simulate RGBD sensor then install opencv for python3 and ros cv bridge:

        sudo -H pip3 install opencv-python
        sudo apt install ros-noetic-cv-bridge

## Test the simulator

We provide with 2 robots for testing purposes: acrobat and r2d2, they can be found [here](https://github.com/oscar-lima/pybullet_ros/tree/noetic/common/test/urdf).

### Bringup r2d2 robot

This code is shipped with a simple URDF robot for testing purposes (r2d2), you can run it by executing:

        roslaunch pybullet_ros bringup_robot_example.launch

You should now be able to visualise the robot in a gui.

### Send position control commands to the robot.

Publish a float msg to the following topic:

        rostopic pub /head_swivel_position_controller/command std_msgs/Float64 "data: 1.0" --once

Move in position control with convenient gui:

        roslaunch pybullet_ros position_cmd_gui.launch

A gui will pop up, use the slides to control the angle of your robot joints in position control.

NOTE: This gui should not be active while sending velocity of effort commands!

### Send joint velocity or effort (torque) control commands to the robot.

NOTE: r2d2 robot has a revolute joint in the neck that can be used to test

position, velocity or effort commands as described below in the following lines:

Before sending commands, make sure position control interface gui publisher is not running!

Publish a float msg to the following topics:

velocity controller interface:

        rostopic pub /head_swivel_velocity_controller/command std_msgs/Float64 "data: 2.0" --once

effort controller interface:

        rostopic pub /head_swivel_effort_controller/command std_msgs/Float64 "data: -2000.0" --once

Done. The robot should now move in velocity or effort control mode with the desired speed/torque.

## Create a map with gmapping 

A modified version of the R2D2 model which is more bottom heavy will be used for this allowing the robot to move faster without toppling 

For this to work you may have to install the following

        sudo apt-get install ros-noetic-turtlebot3-msgs
        sudo apt-get install ros-noetic-turtlebot3
        sudo apt-get install ros-noetic-openslam-gmapping

Steps to run a demo gmapping 

- In a new terminal run 

        roslaunch pybullet_ros move_base_sim.launch 

This will launch pybullet with bottom heavy R2D2 robot and a lab environment

- In a another terminal run the following command to start gmapping

        roslaunch pybullet_ros move_base_gmapping_v1.launch 

- In a new terminal run

        roslaunch pybullet_ros move_base_teleop_v1.launch 

This will start a teleop so that you can move around the robot

- In a new terminal start Rviz with the following command

        roslaunch pybullet_ros move_base_rviz_v1.launch 

This launches Rviz with the requires setting for visualizing. Move the robot around until you have a good map of the area

After the mapping is complete there are two steps to follow

- In Rviz click on file -> save config as -> and save in the folder -> common/rviz

- cd in to the required folder and run the following example command to save the map

        rosrun map_server map_saver -f ~/<directory>/<name of map>

-


## Using move_base for navigation

The setting needed for move_base is given as an example here 

- To begin with run the following command in a new terminal

        roslaunch pybullet_ros move_base_sim.launch 

- The run the map server to start using the map need for navigation

        rosrun map_server map_server <directory>/<map_name>
- Next run run amcl for navigation and localization

        roslaunch pybullet_ros move_base_amcl.launch

- Now run rviz to send 2D goals

        roslaunch pybullet_ros move_base_rviz_amcl.launch 

- Now send 2D navigation goals to the robot. You can use this or use the python move_base client to send navigation goals using the following command

        rosrun pybullet_ros move_base_client_v1.py



## Using MoveIt
- First run the following to install move_it in your system. The example given below is for ros noetic distro


        sudo apt update
        sudo apt upgrade
        sudo apt install ros-noetic-moveit

- In this tutorial we used the the kuka lbr iiwa robot from the source linked below.

        https://github.com/ros-industrial/kuka_experimental

- Next step is to create your own move_it configuration package for the chosen robot. For the purpose of this tutorial we imported kuka robot description made necessary changes to the URDF file and created the move_it configuration package. Clone the repository given below in src folder of your catkin workspace and build the packages.Optionally you are free to choose your own robot and create the necessary move_it configuration package.

        https://github.com/malwaru/kuka_manipulation

- So far the configurations has been set up to work with move_it and pybullet but the controller that connect the action execution between pybullet and moveit is not complete. To see a demonstration with fake controllers and see the the robot in action run the following command

        roslaunch kuka_moveit demo.launch
        
- If the controllers are built use the following commands to see the actual output. Build all packages then open three terminals in each terminal run one of the following launch files

        roslaunch pybullet_ros move_it_sim.launch
        roslaunch kuka_moveit_config move_group.launch
        roslaunch kuka_moveit_config moveit_rviz.launch  

- After running these three are running use Rviz to plan and execute trajectory by adding a MoveIt planner

## Publishing the Depth Image & Point Cloud 
- First launch the pybullet simulator

        roslaunch pybullet_ros move_base_sim.launch

- Then launch Rviz to view the image and point cloud

        roslaunch pybullet_ros depth_image_point_cloud.launch  


## Visualize tf data and robot model in rviz

A convenient configuration file is provided for the visualization of the example robot, run it with:

        rosrun rviz rviz --display-config `rospack find pybullet_ros`/ros/config/pybullet_config.rviz

## Topics you can use to interact with this node

```/joint_states``` (sensor_msgs/JointState) this topic is published at the ```pybullet_ros/loop_rate```
parameter frequency (see parameters section for more detail).
This topic gets listened by the robot state publisher which in turn publishes tf data to the ROS ecosystem.

```/tf``` - This wrapper broadcats all robot transformations to tf, using the robot state publisher and custom plugins.

```/scan```- Using the lidar plugin you get laser scanner readings of type sensor_msgs/LaserScan.

```/odom``` - Using the odometry plugin, robot body odometry gets published (nav_msgs/Odometry).

```/cmd_vel``` - Using the body_vel_control plugin, the robot will subscribe to cmd_vel and exert the desired velocity to the robot.

```<joint_name>_<xtype>_controller/command``` - replace "joint_name" with actual joint name and "xtype"
with [position, velocity, effort] - Using the control plugin, you can publish a joint command on this topic
and the robot will forward the instruction to the robot joint.

```/rgb_image``` - The camera image of type (sensor_msgs/Image)

```/depth_image``` - The depth image of type (sensor_msgs/Image)

```/point_cloud``` - The point cloud of type (sensor_msgs/PointCloud2)

## Services offered by this node

reset simulation, of type std_srvs/Empty, which means it takes no arguments as input, it calls pybullet.resetSimulation() method.

        rosservice call /pybullet_ros/reset_simulation

pause or unpause physics, empty args, prevents the wrapper to call stepSimulation()

        rosservice call /pybullet_ros/pause_physics
        rosservice call /pybullet_ros/unpause_physics

## Parameters

The following parameters can be used to customize the behavior of the simulator.

~ refers to the name of the node (because private nodehandle is used), e.g. pybullet_ros

```~loop_rate``` - Sleep to control the frequency of how often to call pybullet.stepSimulation(), default : 10.0 (hz)

```~pybullet_gui``` - Whether you want to visualize the simulation in a gui or not, default : True

```~robot_urdf_path``` - The path to load a robot at startup, default : None

```~pause_simulation``` - Specify if simulation must start paused (true) or unpaused (false), default : False

```~gravity``` - The desired value of gravity for your simulation physics engine, default : -9.81

```~max_effort``` - The max effort (torque) to apply to the joint while in position or velocity control mode, default: 100.0
                    NOTE: max_effort parameter is ignored when effort commands are given.

```~max_effort_vel_mode``` - Deprecated parameter, use max_effort instead, backwards compatibility is provided, however please change your code asap

```~use_intertia_from_file``` - If True pybullet will compute the inertia tensor based on mass and volume of the collision shape, default: False

```~robot_pose_x``` - The position where to spawn the robot in the world in m, default: 0.0

```~robot_pose_y``` - The position where to spawn the robot in the world in m, default: 0.0

```~robot_pose_z``` - The position where to spawn the robot in the world in m, default: 1.0

```~robot_pose_yaw``` - The orientation where to spawn the robot in the world, default: 0.0

```~fixed_base``` - If true, the first link of the robot will be fixed to the center of the world, useful for non movable robots default: False

```~use_deformable_world``` - Set this paramter to true in case you require soft body simulation, default: False

```~environment``` - The name of the python file (has to be placed inside plugins folder) without the .py extension that implements the necessary
                     custom functions to load an environment via python code, e.g using functions like self.pb.loadURDF(...)
                     See "environment" section below for more details.

```~plugin_import_prefix``` - Allow environment plugins located in external packages to be recognized by pybullet ros. The line executed would look like this:
                              from my_external_ros_pkg.<my environment param (from above)> import Environment # default: pybullet_ros.plugins

```~gui_options``` - Expose gui options to the user, for example to be able to maximize screen -> options="--width=2560 --height=1440".
                    The line of code in pybullet for the presented example would look like this: ```physicsClient = p.connect(p.GUI, options="--width=2560 --height=1440")```

# pybullet ros plugins

What is a pybullet ros plugin?

At the core of pybullet ros, we have the following workflow:

<img src="https://github.com/oscar-lima/pybullet_ros/blob/noetic/common/images/main_loop.png" alt="drawing" width="200"/>

Basically we iterate over all registered plugins, run their execute function, and after doing it for all plugins we step the simulation one time step.

# Plugin creation

This section shows you how you can create your own plugin, to extend this library with your own needs.

NOTE: Before creating a pybullet_ros plugin, make sure your plugin does not exist already
[check available pybullet_ros plugins here](https://github.com/oscar-lima/pybullet_ros/tree/noetic/ros/src/pybullet_ros/plugins).

To ease the process, we provide with a template [here](https://github.com/oscar-lima/pybullet_ros/blob/noetic/ros/src/pybullet_ros/plugins/plugin_template.py).

Copy the template and follow this instructions:

1. roscd pybullet_ros/ros/src/pybullet_ros/plugins && cp plugin_template.py my_awesome_plugin.py

2. add it  to param server

    roscd pybullet_ros/ros/config && gedit pybullet_params_example.yaml

Extend "plugins" param to add yours, e.g:

    plugins: {  pybullet_ros.plugins.body_vel_control: cmdVelCtrl,
                pybullet_ros.plugins.odometry: simpleOdometry,
                pybullet_ros.plugins.control: Control}

    plugins: {  pybullet_ros.plugins.body_vel_control: cmdVelCtrl,
                pybullet_ros.plugins.odometry: simpleOdometry,
                pybullet_ros.plugins.control: Control,
                pybullet_ros.plugins.plugin_template: pluginTemplate}

3. Thats all! you receive a pointer to the robot and to the import of pybullet itself, e.g.

        import pybullet as pb -> self.pb
        self.robot -> self.pb.loadURDF(urdf_path, basePosition=[0,0,0], baseOrientation=[0,0,0,1], ...)

Using the pybullet [documentation](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#) you should be able
to access all the functionality that the pybullet api provides.

# Environment plugin

To load an environment (URDF, SDF, etc) we provide with a particular one time plugin under "plugins/environment.py".

This is loaded during runtime via importlib based upon the value of the ```~environment``` parameter.

The recommended way is to set the "environment" parameter to point to a python file which has to be placed under "plugins" folder (just as any other plugin).

Then set the environment parameter to be a string with the name of your python file, e.g. ```my_env.py``` but without the .py, therefore only : ```my_env```.

Then inside my_env.py inherit from Environment class provided in plugins/environment.py and override the ```load_environment_via_code``` method.

A template is provided under plugins/environment_template.py to ease the process.

As mentioned before, the code inside the method "load_environment_via_code" will be called one time only during puybullet startup sequence.

## NOTE about the multiple r2d2 urdf models in the web

As you might have already noticed, there are multiple r2d2 urdf models in the web, for instance the one that
ROS [uses](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch) to
teach new users about URDF, however is missing collision and inertia tags. Another one can be found under pybullet repo
[data folder](https://github.com/bulletphysics/bullet3/blob/master/data/r2d2.urdf) but that model does not follow
[ROS conventions](https://www.ros.org/reps/rep-0103.html#axis-orientation), in particular "x" axis moves the robot forward and "y" axis moves it to the left.
We have created our own r2d2 and included a lidar on its base to be able to test the laser scanner plugin.

## Wait a second... bullet is already integrated in gazebo. Why do I need this repository at all?

Well thats true, bullet is integrated in gazebo, they have much more plugins available and their api runs much faster as it is implemented in C++.

I myself also use gazebo on a daily basis! , however probably some reasons why this repository be useful are because is very easy and fast to configure a rapid prototype.

Your urdf model does not need to be extended with gazebo tags, installation is extremely easy from pypi and there are lots of examples in pybullet available (see [here](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples)). Additionally, its in python! so is easier and faster to develop + the pybullet documentation is better.
