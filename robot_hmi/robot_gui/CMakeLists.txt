cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  bin/start
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
