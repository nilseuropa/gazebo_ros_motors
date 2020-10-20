cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_motors)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  message_generation
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  rosconsole
  diagnostic_updater
  std_msgs
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

find_package(Boost REQUIRED COMPONENTS thread)

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/motorModel.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  message_runtime
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  rosconsole
  std_msgs
)

###########
## Build ##
###########

add_library(gazebo_ros_dc_motor src/gazebo_ros_dc_motor.cpp)
target_link_libraries(gazebo_ros_dc_motor gazebo_ros_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gazebo_ros_dc_motor ${PROJECT_NAME}_gencfg)

add_library(gazebo_ros_joint_motor src/gazebo_ros_joint_motor.cpp)
target_link_libraries(gazebo_ros_joint_motor gazebo_ros_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##########
## Test ##
##########

# if(CATKIN_ENABLE_TESTING)
# find_package(rostest REQUIRED)
#
# add_rostest_gtest(test_motor_model test/test_motor_model.test src/test/test_motor_model.cpp)
# target_link_libraries(test_motor_model ${catkin_LIBRARIES})
#
# endif()

#############
## Install ##
#############

install(TARGETS
  gazebo_ros_dc_motor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(TARGETS
  gazebo_ros_joint_motor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
