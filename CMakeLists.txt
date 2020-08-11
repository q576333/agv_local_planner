cmake_minimum_required(VERSION 2.8.3)
project(agv_local_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    costmap_2d
    mbf_utility
    mbf_costmap_core
    agv_path_smoothing
    distance_map_node
    pluginlib
    roscpp
    std_msgs
    geometry_msgs
)

find_package(Eigen3 REQUIRED)
#if(Eigen3_FOUND)
  #message("Eigen3 was found!")
  #message("Eigen3 Headers DIRECTORY: " ${EIGEN3_INCLUDE_DIR})
#endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nurbs_controller
  CATKIN_DEPENDS costmap_2d mbf_utility mbf_costmap_core pluginlib roscpp std_msgs geometry_msgs agv_path_smoothing distance_map_node
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(nurbs_controller src/nurbs_planner_ros.cpp src/nurbs_planner.cpp)
add_dependencies(nurbs_controller ${nurbs_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nurbs_controller ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

install(TARGETS nurbs_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/nurbs_controller/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

