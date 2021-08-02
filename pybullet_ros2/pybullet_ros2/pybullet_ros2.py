#!/usr/bin/env python3

import rclpy
import os
import importlib
import pybullet_data

from std_srvs.srv import Empty
from pybullet_ros2.function_exec_manager import FuncExecManager


class pyBulletRosWrapper(object):
    """ROS wrapper class for pybullet simulator"""

    def __init__(self, node):

        self.node = node

        # WZF?!?!?!?
        self.node.declare_parameter("environment")
        self.node.declare_parameter("plugin_import_prefix")
        self.node.declare_parameter("pybullet_gui")
        self.node.declare_parameter("robot_urdf_path")
        self.node.declare_parameter("pause_simulation")
        self.node.declare_parameter("parallel_plugin_execution")
        self.node.declare_parameter("robot_pose_x")
        self.node.declare_parameter("robot_pose_y")
        self.node.declare_parameter("robot_pose_z")
        self.node.declare_parameter("robot_pose_yaw")
        self.node.declare_parameter("fixed_base")
        self.node.declare_parameter("use_deformable_world")
        self.node.declare_parameter("gui_options")
        self.node.declare_parameter("robot_description")
        self.node.declare_parameter("plugins")

        # import pybullet
        self.pb = importlib.import_module('pybullet')
        # get from param server the frequency at which to run the simulation
        self.loop_rate = self.node.declare_parameter('loop_rate', 80.0).value

        # query from param server if gui is needed
        is_gui_needed = self.node.get_parameter('pybullet_gui')

        # get from param server if user wants to pause simulation at startup
        self.pause_simulation = self.node.get_parameter('pause_simulation')

        print('\033[34m')
        # print pybullet stuff in blue
        physicsClient = self.start_gui(gui=is_gui_needed)

        # we dont need to store the physics client for now...
        # setup service to restart simulation
        self.node.create_service(Empty, '/reset_simulation', self.handle_reset_simulation)

        # setup services for pausing/unpausing simulation
        self.node.create_service(Empty, '/pause_physics', self.handle_pause_physics)
        self.node.create_service(Empty, '/unpause_physics', self.handle_unpause_physics)

        # get pybullet path in your system and store it internally for future use, e.g. to set floor
        self.pb.setAdditionalSearchPath(pybullet_data.getDataPath())

        # create object of environment class for later use
        # default : plugins/environment.py
        env_plugin = self.node.get_parameter('environment').value

        plugin_import_prefix = self.node.get_parameter('plugin_import_prefix').value

        print(f'{plugin_import_prefix}.{env_plugin}')
        self.environment = getattr(importlib.import_module(
            f'{plugin_import_prefix}.{env_plugin}'), 'Environment')(self.pb, self.node)

        # load robot URDF model, set gravity, and ground plane
        self.robot = self.init_pybullet_robot()
        self.connected_to_physics_server = None
        if not self.robot:
            self.connected_to_physics_server = False
            return  # Error while loading urdf file
        else:
            self.connected_to_physics_server = True
        # get all revolute joint names and pybullet index

        rev_joint_index_name_dic, prismatic_joint_index_name_dic, fixed_joint_index_name_dic, link_names_to_ids_dic = self.get_properties()
        # import plugins dynamically
        self.plugins = []
        plugins = self.node.get_parameter('plugins').value

        if not plugins:
            node.get_logger().warn('No plugins found, forgot to set param plugins?')
        # return to normal shell color
        print('\033[0m')
        # load plugins
        for plugin in plugins:
            module_, class_ = plugin.split("-")
            print(module_)
            print(class_)

            #  = plugin["class"]
            # params_ = plugin.copy()
            node.get_logger().info('loading plugin: {} class from {}'.format(class_, module_))
            # create object of the imported file class
            obj = getattr(importlib.import_module(module_), class_)(self.pb, self.node, self.robot,
                                                                    rev_joints=rev_joint_index_name_dic,
                                                                    prism_joints=prismatic_joint_index_name_dic,
                                                                    fixed_joints=fixed_joint_index_name_dic,
                                                                    link_ids=link_names_to_ids_dic)
            # store objects in member variable for future use
            self.plugins.append(obj)
        self.node.get_logger().info('pybullet ROS wrapper initialized')

    def get_properties(self):
        """
        construct 3 dictionaries:
        - joint index to joint name x2 (1 for revolute, 1 for fixed joints)
        - link name to link index dictionary
        """
        rev_joint_index_name_dic = {}
        fixed_joint_index_name_dic = {}
        prismatic_joint_index_name_dic = {}
        link_names_to_ids_dic = {}
        for joint_index in range(0, self.pb.getNumJoints(self.robot)):
            info = self.pb.getJointInfo(self.robot, joint_index)
            # build a dictionary of link names to ids
            link_names_to_ids_dic[info[12].decode('utf-8')] = joint_index
            # ensure we are dealing with a revolute joint
            if info[2] == self.pb.JOINT_REVOLUTE:
                # insert key, value in dictionary (joint index, joint name)
                rev_joint_index_name_dic[joint_index] = info[1].decode('utf-8')  # info[1] refers to joint name
            elif info[2] == self.pb.JOINT_FIXED:
                # insert key, value in dictionary (joint index, joint name)
                fixed_joint_index_name_dic[joint_index] = info[1].decode('utf-8')  # info[1] refers to joint name
            elif info[2] == self.pb.JOINT_PRISMATIC:
                prismatic_joint_index_name_dic[joint_index] = info[1].decode('utf-8')  # info[1] refers to joint name
        return rev_joint_index_name_dic, prismatic_joint_index_name_dic, fixed_joint_index_name_dic, link_names_to_ids_dic

    def handle_reset_simulation(self, req):
        """Callback to handle the service offered by this node to reset the simulation"""
        self.node.get_logger().info('reseting simulation now')
        self.pb.resetSimulation()
        return Empty()

    def start_gui(self, gui=True):
        """start physics engine (client) with or without gui"""
        if(gui):
            # start simulation with gui
            self.node.get_logger().info('Running pybullet with gui')
            self.node.get_logger().info('-------------------------')
            # e.g. to maximize screen: options="--width=2560 --height=1440"
            gui_options = self.node.get_parameter('gui_options').value
            return self.pb.connect(self.pb.GUI, options=gui_options)
        else:
            # start simulation without gui (non-graphical version)
            self.node.get_logger().info('Running pybullet without gui')
            # hide console output from pybullet
            self.node.get_logger().info('-------------------------')
            return self.pb.connect(self.pb.DIRECT)

    def init_pybullet_robot(self):
        """load robot URDF model, set gravity, ground plane and environment"""
        # get from param server the path to the URDF robot model to load at startup
        urdf_path = self.node.get_parameter('robot_urdf_path').value
        if urdf_path == None:
            self.node.get_logger().error('mandatory param robot_urdf_path not set, will exit now')
            rclpy.shutdown()

        # test urdf file existance
        if not os.path.isfile(urdf_path):
            self.node.get_logger().error('param robot_urdf_path is set, but file does not exist : ' + urdf_path)
            self.node.get_logger().error('required robot urdf file not found')
            rclpy.shutdown()
            return None
        # ensure urdf is not echeeeeeeee, but if it is then make urdf file version out of it
        if 'xacro' in urdf_path:
            robot_description = self.node.get_parameter('robot_description').value
            if not robot_description:
                self.node.get_logger().error('required robot_description param not set')
                return None
            # remove xacro from name
            urdf_path_without_xacro = urdf_path[0:urdf_path.find(
                '.xacro')]+urdf_path[urdf_path.find('.xacro')+len('.xacro'):]
            self.node.get_logger().info('generating urdf model from xacro from robot_description param server under: {0}'.format(
                urdf_path_without_xacro))
            try:
                urdf_file = open(urdf_path_without_xacro, 'w')
            except:
                self.node.get_logger().error('Failed to create urdf file from xacro, cannot write into destination: {0}'.format(
                    urdf_path_without_xacro))
                return None
            urdf_file.write(robot_description)
            urdf_file.close()
            urdf_path = urdf_path_without_xacro
        # get robot spawn pose from parameter server

        robot_pose_x = self.node.get_parameter('robot_pose_x').value
        robot_pose_y = self.node.get_parameter('robot_pose_y').value
        robot_pose_z = self.node.get_parameter('robot_pose_z').value
        robot_pose_yaw = self.node.get_parameter('robot_pose_yaw').value

        robot_spawn_orientation = self.pb.getQuaternionFromEuler([0.0, 0.0, robot_pose_yaw])
        fixed_base = self.node.get_parameter('fixed_base').value
        # load robot from URDF model
        # user decides if inertia is computed automatically by pybullet or custom
        if self.node.declare_parameter('use_intertia_from_file', False):
            # combining several boolean flags using "or" according to pybullet documentation
            urdf_flags = self.pb.URDF_USE_INERTIA_FROM_FILE | self.pb.URDF_USE_SELF_COLLISION
        else:
            urdf_flags = self.pb.URDF_USE_SELF_COLLISION
        # load environment
        self.node.get_logger().info('loading environment')
        self.environment.load_environment()
        # set no realtime simulation, NOTE: no need to stepSimulation if setRealTimeSimulation is set to 1
        self.pb.setRealTimeSimulation(0)  # NOTE: does not currently work with effort controller, thats why is left as 0
        self.node.get_logger().info('loading urdf model: ' + urdf_path)

        # NOTE: self collision enabled by default

        return self.pb.loadURDF(urdf_path, basePosition=[robot_pose_x, robot_pose_y, robot_pose_z],
                                baseOrientation=robot_spawn_orientation,
                                useFixedBase=fixed_base, flags=urdf_flags)

    def handle_reset_simulation(self, req):
        """Callback to handle the service offered by this node to reset the simulation"""
        self.node.get_logger().info('reseting simulation now')
        # pause simulation to prevent reading joint values with an empty world
        self.pause_simulation = True
        # remove all objects from the world and reset the world to initial conditions
        self.pb.resetSimulation()
        # load URDF model again, set gravity and floor
        self.init_pybullet_robot()
        # resume simulation control cycle now that a new robot is in place
        self.pause_simulation = False
        return []

    def handle_pause_physics(self, req):
        """pause simulation, raise flag to prevent pybullet to execute self.pb.stepSimulation()"""
        self.node.get_logger().info('pausing simulation')
        self.pause_simulation = False
        return []

    def handle_unpause_physics(self, req):
        """unpause simulation, lower flag to allow pybullet to execute self.pb.stepSimulation()"""
        self.node.get_logger().info('unpausing simulation')
        self.pause_simulation = True
        return []

    def pause_simulation_function(self):
        return self.pause_simulation

    def start_pybullet_ros_wrapper_sequential(self):
        """
        This function is deprecated, we recommend the use of parallel plugin execution
        """
        rate = self.node.create_rate(self.loop_rate)
        while rclpy.ok:
            if not self.pause_simulation:
                # run x plugins
                for task in self.plugins:
                    task.execute()
                # perform all the actions in a single forward dynamics simulation step such
                # as collision detection, constraint solving and integration
                self.pb.stepSimulation()
            rate.sleep()
        self.node.get_logger().warn('killing node now...')
        # if node is killed, disconnect
        if self.connected_to_physics_server:
            self.pb.disconnect()

    def start_pybullet_ros_wrapper_parallel(self):
        """
        Execute plugins in parallel, however watch their execution time and warn if exceeds the deadline (loop rate)
        """
        # create object of our parallel execution manager
        exec_manager_obj = FuncExecManager(self.plugins, rclpy.ok, self.pb.stepSimulation, self.pause_simulation_function,
                                           log_info=self.node.get_logger().info, log_warn=self.node.get_logger().warn, log_debug=self.node.get_logger().info, function_name='plugin')
        # start parallel execution of all "execute" class methods in a synchronous way
        exec_manager_obj.start_synchronous_execution(loop_rate=self.loop_rate)
        # ctrl + c was pressed, exit
        self.node.get_logger().warn('killing node now...')
        # if node is killed, disconnect
        if self.connected_to_physics_server:
            self.pb.disconnect()

    def start_pybullet_ros_wrapper(self):
        if self.node.get_parameter('parallel_plugin_execution'):
            self.start_pybullet_ros_wrapper_parallel()
        else:
            self.start_pybullet_ros_wrapper_sequential()


def main():
    rclpy.init()
    node = rclpy.create_node("pybullet_ros2")

    node.get_logger().info("Created node, 'pybullet_ros2'")

    pybullet_ros_interface = pyBulletRosWrapper(node)
    pybullet_ros_interface.start_pybullet_ros_wrapper()
