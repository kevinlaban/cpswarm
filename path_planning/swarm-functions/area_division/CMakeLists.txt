cmake_minimum_required(VERSION 3.0.2)
project(area_division)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES area_division_lib
  CATKIN_DEPENDS roscpp std_msgs nav_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(area_division_lib
  src/lib/area_division.cpp
  src/lib/connected_components.cpp
)

add_executable(area_division_node src/main.cpp)
target_link_libraries(area_division_node
  area_division_lib
  ${catkin_LIBRARIES}
)

## Add executable for the testbench
add_executable(test_pad_occupancy_grid src/test_pad_occupancy_grid.cpp)
add_dependencies(test_pad_occupancy_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pad_occupancy_grid
  ${catkin_LIBRARIES}
)