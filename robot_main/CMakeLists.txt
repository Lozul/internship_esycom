cmake_minimum_required(VERSION 3.0.2)
project(robot_main)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robot_driver
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Global.cfg
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
