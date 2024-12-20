cmake_minimum_required(VERSION 2.8.3)
project(swarm_bridge)

set(CMAKE_BUILD_TYPE "Debug")
# ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

set(ENABLE_TCP true) # requires zmq, zmqpp 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  serial
  traj_utils
  traj_opt # for traj2odom_node
  sensor_msgs
  quadrotor_msgs
  message_generation
  mission_msgs
  ground_station_msgs
)
add_message_files(
  FILES
  correction.msg
  battary_velocity.msg
  heartbeat.msg
  gps.msg
  MiniMINCO.msg
  DroneState.msg
  MiniPoly.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
 CATKIN_DEPENDS traj_utils
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(bridge_node_udp
  src/bridge_node_udp.cpp 
  )
target_link_libraries(bridge_node_udp 
  ${catkin_LIBRARIES}
  )

# add_executable(goal_trans
#   src/goal_trans.cpp 
#   )
# target_link_libraries(goal_trans 
#   ${catkin_LIBRARIES}
#   )

if(ENABLE_TCP)

  add_executable(bridge_node_tcp
    src/bridge_node_tcp.cpp 
    )
  target_link_libraries(bridge_node_tcp
    ${catkin_LIBRARIES}
    zmq
    zmqpp
    )

endif(ENABLE_TCP)

add_executable(traj2odom_node
  src/traj2odom_node.cpp 
  )
target_link_libraries(traj2odom_node 
  ${catkin_LIBRARIES}
  )

catkin_install_python(
  PROGRAMS scripts/goal_tran.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
add_executable(telemetry_air2ground
  src/telemetry_air2ground.cpp
)
target_link_libraries(telemetry_air2ground
  ${catkin_LIBRARIES}
)
add_executable(bridge_node_uwb
  src/bridge_node_uwb.cpp
)
target_link_libraries(bridge_node_uwb
  ${catkin_LIBRARIES}
)