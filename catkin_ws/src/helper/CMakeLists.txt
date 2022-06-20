cmake_minimum_required(VERSION 3.0.2)
project(helper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS
CATKIN_DEPENDS roscpp
                pcl_conversions
                pcl_ros
)
add_executable(cluster_extraction src/cluster_extraction.cpp)
target_link_libraries(cluster_extraction ${catkin_LIBRARIES})
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/cluster2obj.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



