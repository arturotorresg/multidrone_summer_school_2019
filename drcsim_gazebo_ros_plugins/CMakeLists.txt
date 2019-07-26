cmake_minimum_required(VERSION 2.8.3)
project(drcsim_gazebo_ros_plugins)
set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Check for 32/64 bits support
STRING (REGEX MATCH "Linux" LINUX ${CMAKE_SYSTEM_NAME})
STRING (REGEX MATCH "BSD" BSD ${CMAKE_SYSTEM_NAME})

find_package(catkin REQUIRED COMPONENTS
             drcsim_model_resources
             drcsim_gazebo_plugins
             roscpp
             gazebo_plugins
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drcsim_gazebo_ros_plugins
  CATKIN_DEPENDS gazebo_plugins
#  DEPENDS system_lib
)

find_package(gazebo REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Declare a cpp library
add_library(DRCVehicleROSPlugin src/DRCVehicleROSPlugin.cpp)
target_link_libraries(DRCVehicleROSPlugin ${catkin_LIBRARIES})
#add_dependencies(DRCVehicleROSPlugin DRCVehiclePlugin)

#############
## Install ##
#############
install(TARGETS
  DRCVehicleROSPlugin
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
