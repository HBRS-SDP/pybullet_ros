cmake_minimum_required(VERSION 2.8.3)
project(pybullet_ros)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    cv_bridge
)

# catkin_install_python(PROGRAMS scripts/talker.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )



catkin_python_setup()
