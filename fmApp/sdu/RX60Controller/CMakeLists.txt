cmake_minimum_required(VERSION 2.8.3)
project(rx60controller)

find_package(catkin REQUIRED COMPONENTS 
roscpp 
rospy 
std_msgs 
message_generation
)


add_service_files(FILES command.srv)

generate_messages(DEPENDENCIES std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

#### rx60 driver ###
set(SrcFiles
  src/rx60_controller/RX60Driver.cpp
  src/rx60_controller/RX60TCPSocket.cpp)

#link_directories( )

set(LibraryList
  pthread)
#  curses)

# The shared library to build:

# Link the standard static libraries with with the shared library:
#target_link_libraries(RX60 ${LibraryList})

add_executable(rx60_controller src/rx60_controller/rx60_controller_node.cpp							   src/rx60_controller/RX60Driver.cpp
				src/rx60_controller/RX60TCPSocket.cpp)

target_link_libraries(rx60_controller ${catkin_LIBRARIES})
add_dependencies(rx60_controller system thread)

add_executable(rx60_test src/rx60_controller/nodetest.cpp src/rx60_controller/RX60Driver.cpp src/rx60_controller/RX60TCPSocket.cpp)

target_link_libraries(rx60_test ${catkin_LIBRARIES})
add_dependencies(rx60_test system thread)

#target_link_libraries(talker ${catkin_LIBRARIES})
#add_executable(test-RX60 src/rx60_controller/test.cpp ${SrcFiles})
#target_link_libraries(test-RX60 ${catkin_LIBRARIES})
#add_dependencies(test-RX60 system thread)

#add_executable(nodetest-RX60 src/rx60_controller/nodetest.cpp ${SrcFiles})
#target_link_libraries(nodetest-RX60 ${catkin_LIBRARIES})
#add_dependencies(nodetest-RX60 system thread)

### end of rx60 driver ###

# %EndTag(FULLTEXT)%
