cmake_minimum_required(VERSION 2.8.3)
project(localisation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall) 

find_package(catkin REQUIRED COMPONENTS
  dead_reckoning
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Armadillo REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  dead_reckoning 
  roscpp 
  std_msgs 
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME} 
  src/main.cpp
  src/particle_filter.cpp
)

add_dependencies(
  ${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${ARMADILLO_INCLUDE_DIRS}
  -llapack 
  -lopenblas
  -larmadillo
)

