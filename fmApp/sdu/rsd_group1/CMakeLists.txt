cmake_minimum_required(VERSION 2.8.3)
project(rsd_group1)

SET(RW_ROOT "/home/anoch/RWPROJECTROOT/RobWork")

SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})
FIND_PACKAGE(RobWork)
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )



find_package(catkin REQUIRED COMPONENTS
  qt_build
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  message_generation
  std_msgs
  wsg_50_common
)

 
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  mes_sorting_command.msg
  mes_sorting_status.msg
  Log.msg
  Num.msg
  OEEmsg.msg
  mes_order.msg
  lego_brick.msg
  Log.msg
  general.msg
  msg_hi.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

rosbuild_prepare_qt4(QtCore QtGui)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} HMI/ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} HMI/resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS HMI/include/HMI/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS HMI/src/*.cpp)

include_directories(HMI ${catkin_INCLUDE_DIRS} ${ROBWORK_INCLUDE_DIR})

add_executable(HMI ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(HMI ${QT_LIBRARIES} ${catkin_LIBRARIES}  ${ROBWORK_LIBRARIES})
install(TARGETS HMI RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(robotics Robotics/src/robotics.cpp Robotics/src/robot.cpp)
target_link_libraries(robotics ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES})

add_executable(conveyor_node Conveyor/src/conveyor.cpp)
target_link_libraries(conveyor_node ${catkin_LIBRARIES})
