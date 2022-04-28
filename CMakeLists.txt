cmake_minimum_required(VERSION 2.8.3)
project(graph_tools)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp hdl_graph_slam roslib)
find_package(G2O REQUIRED)

catkin_package(
)

# odom to graph
set(ODOM_2_GRAPH_SOURCES 
  src/odometry_to_graph/odometry_to_graph.cpp 
  src/odometry_to_graph/OdometryFrame.cpp 
  src/odometry_to_graph/OdometrySet.cpp 
)

add_executable (odometry_to_graph ${ODOM_2_GRAPH_SOURCES})
target_include_directories (odometry_to_graph PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/odometry_to_graph
  ${G2O_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(odometry_to_graph
  ${PCL_LIBRARIES}
  ${G2O_CORE_LIBRARY}
  ${G2O_TYPES_SLAM3D}
  ${catkin_LIBRARIES}
)

# merge maps
set(MERGE_MAP_SOURCES
  src/merge_map/Graph.cpp 
  src/merge_map/merge_map.cpp 
)

add_executable (merge_maps ${MERGE_MAP_SOURCES})
target_include_directories (merge_maps PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/merge_map
  ${G2O_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(merge_maps
  ${PCL_LIBRARIES}
  ${G2O_CORE_LIBRARY}
  ${G2O_TYPES_SLAM3D}
  ${catkin_LIBRARIES}
)