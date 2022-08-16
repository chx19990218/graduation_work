cmake_minimum_required(VERSION 3.0.2)
project(mpcc_solver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)
find_package(Eigen3 REQUIRED)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include
)
add_executable(mpcc_node 
  src/main.cpp
)
target_link_libraries(mpcc_node
  ${catkin_LIBRARIES}
)


