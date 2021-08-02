find_package(rosidl_default_generators REQUIRED)
find_package(xacro REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Empty.srv"
 )

