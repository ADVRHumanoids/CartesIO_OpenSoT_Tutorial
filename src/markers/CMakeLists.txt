cmake_minimum_required(VERSION 2.8.3)
project(markers)

# C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  interactive_markers
  tf
)

add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})

add_executable(points_and_lines src/points_and_lines.cpp)
target_link_libraries(points_and_lines ${catkin_LIBRARIES})

add_executable(simple_marker src/simple_marker.cpp)
target_link_libraries(simple_marker ${catkin_LIBRARIES})

add_executable(basic_controls src/basic_controls.cpp)
target_link_libraries(basic_controls ${catkin_LIBRARIES})

add_executable(menu src/menu.cpp)
target_link_libraries(menu ${catkin_LIBRARIES})
