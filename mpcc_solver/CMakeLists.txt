cmake_minimum_required(VERSION 3.0.2)
project(mpcc_solver)

find_package(catkin REQUIRED COMPONENTS
  quadrotor_msgs
  geometry_msgs
  roscpp
  roslib
  std_msgs
  mavros
)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)

catkin_package(

)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include
  src/Spline
  src/map
  src/mpcc
  src/referenceline
  src/matplotlib-cpp
  src/utils
  src/plot
  src/obstacle
  src/test
  src/config
  src/log
  /usr/local/include/osqp
)
add_executable(mpcc_node 
  src/main.cpp
  src/Spline/arc_length_spline.cpp
  src/Spline/cubic_spline.cpp
  src/map/map.cpp
  src/mpcc/mpcc.cpp
  src/plot/plot.cpp
  src/mpcc/constrains.cpp
  src/mpcc/chance_constrains.cpp
  src/mpcc/osqp_interface.cpp
  src/obstacle/obstacle.cpp
  src/referenceline/search.cpp
  src/referenceline/smooth.cpp
  src/referenceline/resample.cpp
  src/log/log.cpp
)

target_link_libraries(mpcc_node ${PYTHON_LIBRARIES})

target_link_libraries(mpcc_node
  ${catkin_LIBRARIES}
  osqp
)


