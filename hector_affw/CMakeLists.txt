cmake_minimum_required(VERSION 2.8.3)
project(hector_affw)

include(FindProtobuf)
find_package( Protobuf REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  affw_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp affw_msgs nav_msgs tf
)

add_executable(hector_affw src/hector_affw.cpp)
target_link_libraries(hector_affw ${catkin_LIBRARIES})
add_dependencies(hector_affw affw_msgs_generate_messages_cpp)
