cmake_minimum_required(VERSION 2.8.3)
project(ljhn_torcon)

#SET(CXX_DISABLE_WERROR True)
#set(DOXYGEN_USE_MATHJAX "YES")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++0x -pedantic")
set(CMAKE_BUILD_TYPE Release)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  angles
  motion_interface
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ljhn_torcon
#   CATKIN_DEPENDS roscpp std_msgs
#   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/imuDatafusion
  ${CMAKE_CURRENT_SOURCE_DIR}/include/LIPM/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/DCM/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/bhuman/

  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

#add_definitions( -mfpmath=sse -msse2)
add_definitions( -std=c++14)


#ADD_REQUIRED_DEPENDENCY("sch-core" "sch-core_d")
#ADD_REQUIRED_DEPENDENCY("SpaceVecAlg")
#ADD_REQUIRED_DEPENDENCY("RBDyn" "RBDyn_d")
#ADD_REQUIRED_DEPENDENCY("eigen-qld")
#ADD_OPTIONAL_DEPENDENCY("eigen-lssol")

aux_source_directory(./include/imuDatafusion DIR_SRCS)
aux_source_directory(./src DIR_SRCS)
aux_source_directory(./src/bhuman/ DIR_SRCS)
aux_source_directory(./src/DCM/ DIR_SRCS)
add_executable(${PROJECT_NAME}_node
  ${DIR_SRCS}  
)
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node 
  PUBLIC SpaceVecAlg RBDyn eigen-qld sch-core Tasks osqp
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES}
  yaml-cpp
)

