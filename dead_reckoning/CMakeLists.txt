cmake_minimum_required(VERSION 2.8.3)
project(dead_reckoning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  phidgets
  geometry_msgs
  wheel_sensor
  tf
)

catkin_package(
  CATKIN_DEPENDS 
  phidgets 
  roscpp  
  std_msgs 
  geometry_msgs 
  wheel_sensor 
  tf
)

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/dead_reckoning.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})



