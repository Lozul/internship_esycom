cmake_minimum_required(VERSION 3.0.2)
project(robot_main)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robot_driver
  dynamic_reconfigure
  message_generation
)

add_message_files(
    FILES
    Routine.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Global.cfg
)

catkin_package(
    CATKIN_DEPENDS message_runtime    
)

include_directories(
  include
  /usr/include
  ${catkin_INCLUDE_DIRS}
)

# add_library(lmshidLib STATIC IMPORTED)
# set_target_properties(lmshidLib PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/LMShid.so)

add_executable(${PROJECT_NAME}_node src/main.cpp src/LMShid.c)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  usb
)

# add_executable(${PROJECT_NAME}_test_node src/test.cpp)
# target_link_libraries(${PROJECT_NAME}_test_node
#     ${catkin_LIBRARIES}
# )
