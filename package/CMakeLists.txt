cmake_minimum_required(VERSION 2.8.3)
project(multi_sensor_fusion )

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)

# Directories definition
set(MULTI_SENSOR_FUSION_SOURCE_DIR
  src/source
)

set(MULTI_SENSOR_FUSION_INCLUDE_DIR
  src/include
)

set(MULTI_SENSOR_FUSION_HEADER_FILES
        ${MULTI_SENSOR_FUSION_INCLUDE_DIR}/behavior_self_localize_with_ekf_sensor_fusion.h  
)

set(MULTI_SENSOR_FUSION_SOURCE_FILES
        ${MULTI_SENSOR_FUSION_SOURCE_DIR}/behavior_self_localize_with_ekf_sensor_fusion.cpp
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  behavior_execution_controller
  sensor_msgs
  tf_conversions
  droneMsgsROS
  tf
  geometry_msgs
  aerostack_msgs
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
    message(WARN "Using Eigen2!")
else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${MULTI_SENSOR_FUSION_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  nodelet
  std_msgs
  sensor_msgs
  behavior_execution_controller
  tf_conversions
  geometry_msgs
  tf
  droneMsgsROS
  aerostack_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(multi_sensor_fusion 
  src/source/behavior_self_localize_with_ekf_sensor_fusion.cpp
)
target_link_libraries(multi_sensor_fusion  ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(multi_sensor_fusion  ${catkin_EXPORTED_LIBRARIES})
endif()
