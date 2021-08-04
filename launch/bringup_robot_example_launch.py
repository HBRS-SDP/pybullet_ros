from os.path import join
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():

    path_to_urdf = join(get_package_share_directory('pybullet_ros2'), 'r2d2.urdf.xacro')

    return LaunchDescription([

        Node(
            package='pybullet_ros2',
            executable='pybullet_ros2',
            output='screen',
            parameters=[
                    {"plugin_import_prefix": "pybullet_ros2.plugins"},
                    {"environment": "environment"},
                    {"pybullet_gui": True},
                    {"robot_urdf_path": path_to_urdf},
                    {"pause_simulation": False},
                    {"parallel_plugin_execution": True},
                    {"robot_pose_x": 0.0},
                    {"robot_pose_y": 0.0},
                    {"robot_pose_z": 0.7},
                    {"robot_pose_yaw": 0.0},
                    {"fixed_base": False},
                    {"use_deformable_world": False},
                    {"gui_options": ""},
                    {'robot_description': Command(['xacro', ' ', path_to_urdf])},
                    # WTF WHY CANT I USE BETTER DATATYPES?!?!?!?!!?!?!?
                    {'plugins': [
                        "pybullet_ros2.plugins.body_vel_control-cmdVelCtrl",
                        "pybullet_ros2.plugins.odometry-simpleOdometry",
                        "pybullet_ros2.plugins.control-Control",
                        "pybullet_ros2.plugins.joint_state_pub-joinStatePub",
                        "pybullet_ros2.plugins.laser_scanner-laserScanner"
                        # "pybullet_ros2.plugins.rgbd_camera-RGBDCamera"
                    ]}

            ]
        ),
    ])
