cmake_minimum_required(VERSION 3.0.2)
project(ground_finder)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  ikd_tree
  pcl_conversions
  tf2_ros
  tf2
)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ground_finder
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ----------------- MAIN SCRPIT --------------------
## Declare a C++ executable
add_executable(ground_finder_node src/main.cpp src/ground_finder.cpp src/math_helpers.cpp src/hough.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(ground_finder_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ----------------- TEST SCRPITS --------------------
## Declare a C++ executable
add_executable(test_node src/test_files/test.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Declare a C++ executable
add_executable(ikd_test_node src/test_files/ikd_tree_test.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(ikd_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Declare a C++ executable
add_executable(pcl_kd_test_node src/test_files/pcl_kd_tree_test.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(pcl_kd_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Declare a C++ executable
add_executable(plane_test_node src/test_files/plane_test.cpp src/math_helpers.cpp src/hough.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(plane_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Declare a C++ executable
add_executable(cropbox_test_node src/test_files/cropbox_test.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(cropbox_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
