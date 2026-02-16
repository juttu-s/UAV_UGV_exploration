cmake_minimum_required(VERSION 3.0.2)
project(uav_ugv_coordinator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  mavros_msgs
  autonomous_flight
  tracking_controller
  global_planner
  map_manager
  trajectory_planner
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(uav_ugv_coordinator src/uav_ugv_coordinator.cpp)
target_link_libraries(uav_ugv_coordinator ${catkin_LIBRARIES})
add_dependencies(uav_ugv_coordinator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS uav_ugv_coordinator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
