cmake_minimum_required(VERSION 2.8.3)
project(test)

# C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  interactive_markers
  tf
)
find_package(cartesian_interface REQUIRED)
find_package(OpenSoT REQUIRED)
find_package(XBotInterface REQUIRED)



include_directories(/usr/include/eigen3)   

include_directories(${cartesian_interface_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(test1 src/test1.cpp)

target_link_libraries(test1 ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES})

add_executable(cartesio_solver src/cartesio_solver.cpp)
target_compile_definitions(cartesio_solver PRIVATE
    -DURDF_PATH="/home/user/MultiDoF-superbuild/configs/CogimonConfig/urdf/cogimon.urdf"
    -DSRDF_PATH="/home/user/MultiDoF-superbuild/configs/CogimonConfig/srdf/cogimon.srdf"
    -DIK_PB_PATH="/home/user/MultiDoF-superbuild/configs/CogimonConfig/cogimon_stack.yaml")
target_link_libraries(cartesio_solver ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES})

add_executable(walking_solver src/walking_solver.cpp)
target_compile_definitions(walking_solver PRIVATE
    -DURDF_PATH="/home/user/MultiDoF-superbuild/configs/CogimonConfig/urdf/cogimon.urdf"
    -DSRDF_PATH="/home/user/MultiDoF-superbuild/configs/CogimonConfig/srdf/cogimon.srdf"
    -DIK_PB_PATH="/home/user/MultiDoF-superbuild/configs/CogimonConfig/cogimon_stack2.yaml")
target_link_libraries(walking_solver ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES})

add_executable(OpenSoT src/OpenSoT.cpp)

target_link_libraries(OpenSoT ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${XBotInterface_LIBRARIES})



add_executable(cartesio_solver_marker src/cartesio_solver_marker.cpp)
target_compile_definitions(cartesio_solver_marker PRIVATE
    -DURDF_PATH="/home/user/MultiDoF-superbuild/configs/CogimonConfig/urdf/cogimon.urdf"
    -DSRDF_PATH="/home/user/MultiDoF-superbuild/configs/CogimonConfig/srdf/cogimon.srdf"
    -DIK_PB_PATH="/home/user/MultiDoF-superbuild/configs/CogimonConfig/cogimon_stack.yaml")
target_link_libraries(cartesio_solver_marker ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES})

