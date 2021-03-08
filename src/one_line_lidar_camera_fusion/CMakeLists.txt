cmake_minimum_required(VERSION 3.0.2)
project(one_line_lidar_camera_fusion)


add_definitions(-std=c++11 -O2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(
  OpenCV  REQUIRED
)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
  cv_bridge
  std_msgs
  image_transport
  message_generation
  
  
  std_msgs
  jsk_recognition_msgs

  geometry_msgs

  #darknet_ros
  darknet_ros_msgs
  perception_msgs
  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

find_package(PCL 1.7 REQUIRED)


add_message_files(
  FILES
  BoundingBoxWithDist.msg
  BoundingBoxesWithDist.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#add_dependencies(darknet_ros_msgs darknet_ros_msgs_gencpp)

catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs roscpp 
  rospy 
  std_msgs 
  message_runtime
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  

)




## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )





## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#add_executable(${PROJECT_NAME}_node src/My_Filter.cpp src/my_pcl_node.cpp)
add_executable(${PROJECT_NAME}_node src/single_line_projection.cpp src/lidar_camera_fusion_node.cpp)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)





