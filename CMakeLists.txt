cmake_minimum_required(VERSION 3.0.2)
project(smart_gui)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosgraph_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/smart_gui_api
  scripts/random_int8_topics
  scripts/int_topics_and_logs
  scripts/smart_gui_frontend_runner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
