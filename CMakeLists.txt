cmake_minimum_required(VERSION 3.0.2)
project(monash_perception)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    roslib
    image_transport
    cv_bridge
    sensor_msgs
    stereo_msgs
    image_geometry
    pcl_ros
)
find_package(OpenCV 3 REQUIRED)

# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   tf
#   sensor_msgs
#   cv_bridge
#   std_msgs
#   image_transport
#   mavros_msgs
# )

# find_package(OpenCV REQUIRED COMPONENTS core calib3d)


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs
    DEPENDS OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} 
  src/t265_depth_node.cpp 
  src/t265_depth.cpp
)

# add_executable(t265_undistort_node src/t265_undistort.cpp)
# target_link_libraries(t265_undistort_node
#   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
# )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries( ${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
)
