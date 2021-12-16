cmake_minimum_required(VERSION 3.0.2)
project(robot_driver)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    laser_geometry
    tf
    message_generation
)

add_service_files(
    FILES
    GetCorrection.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/robot_driver.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/get_correction_server.py
  scripts/test_correction_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
