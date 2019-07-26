cmake_minimum_required(VERSION 2.8.3)
project(summer_school_exercises)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	gazebo_msgs
	geometry_msgs
	roscpp
	sensor_msgs
	nav_msgs
	eigen_conversions
	tf
  uav_abstraction_layer
)

find_package(Eigen3 REQUIRED)
#find_package(gazebo REQUIRED)

# Check for mavros version for compatibility with older versions
execute_process(COMMAND rosversion mavros
		OUTPUT_VARIABLE mavros_version
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
if ("${mavros_version}" VERSION_LESS "0.25.0")
  add_definitions("-DMAVROS_VERSION_BELOW_0_25_0=\"${mavros_version}\"")
endif()

catkin_package(
    INCLUDE_DIRS #${Eigen3_INCLUDE_DIRS}
    CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp sensor_msgs eigen_conversions tf uav_abstraction_layer nav_msgs
)


###########
## BUILD ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")
# -Wno-deprecated-declarations: Shut up warnings about std::binder1st, std::binder2nd.

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(demo_follow_vehicle_ual_server src/demo_follow_vehicle_ual_server.cpp)
target_link_libraries(demo_follow_vehicle_ual_server ${catkin_LIBRARIES})

add_executable(drc_vehicle_moving src/drc_vehicle_moving.cpp)
target_link_libraries(drc_vehicle_moving ${catkin_LIBRARIES})
