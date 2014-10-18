cmake_minimum_required(VERSION 2.8.3)
project(rsd_group1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robotics src/robotics.cpp)
target_link_libraries(robotics ${catkin_LIBRARIES})

add_executable(hmi src/hmi.cpp)
target_link_libraries(hmi ${catkin_LIBRARIES})

add_executable(conveyor_test_node src/conveyor_test.cpp)
target_link_libraries(conveyor_test_node ${catkin_LIBRARIES})
