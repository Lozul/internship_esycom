cmake_minimum_required(VERSION 3.0.2)
project(robot_cli)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  bin/cli
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)