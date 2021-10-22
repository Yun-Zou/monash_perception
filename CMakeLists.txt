cmake_minimum_required(VERSION 3.0.2)
project(monash_perception)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  mavros_msgs
  visualization_msgs
  geometry_msgs
  apriltag_ros
  turtlesim
)
# find_package(OpenCV 3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)

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
    # INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs
    DEPENDS OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} 
  src/apriltag_pose.cpp 
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

add_executable(t265_fisheye_undistort src/t265_fisheye_undistort.cpp)
target_link_libraries(t265_fisheye_undistort
  ${catkin_LIBRARIES}
)

add_executable(apriltag_pose_node src/apriltag_pose.cpp)
target_link_libraries(apriltag_pose_node
  ${catkin_LIBRARIES}
)

add_executable(fake_pose src/fake_pose.cpp)
target_link_libraries(fake_pose
  ${catkin_LIBRARIES}
)

add_executable(apriltag_tf src/apriltag_tf.cpp)
target_link_libraries(apriltag_tf
  ${catkin_LIBRARIES}
)

add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster
  ${catkin_LIBRARIES}
)

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})

add_executable(frame_tf_broadcaster src/frame_tf_broadcaster.cpp)
target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY
#   launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vision_to_mavros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
