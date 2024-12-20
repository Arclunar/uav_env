cmake_minimum_required(VERSION 3.0.2)
project(merge_height)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  nlink_parser
)


catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(vio_fake src/vio_fake.cpp)
target_link_libraries(vio_fake
  ${catkin_LIBRARIES}
)
add_dependencies(vio_fake ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(height_fusion src/height_fusion.cpp)
target_link_libraries(height_fusion
  ${catkin_LIBRARIES}
)
add_dependencies(height_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(height_fusion_mocap src/height_fusion_mocap.cpp)
target_link_libraries(height_fusion_mocap
  ${catkin_LIBRARIES}
)
add_dependencies(height_fusion_mocap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

