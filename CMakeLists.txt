cmake_minimum_required(VERSION 3.4.0)
set(PROJECT_NAME gates_path_generator)
project(${PROJECT_NAME} VERSION 1.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  mavros_msgs
  mav_msgs
  robot_process
  tf
  tf2
  ros_utils_lib
)

catkin_package(
  INCLUDE_DIRS src/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs mavros_msgs robot_process mav_msgs tf tf2 ros_utils_lib
  )

include_directories(
  src/include
  ${catkin_INCLUDE_DIRS}
)


set(CPP_PROJECT_FILES 
  src/source/gates_path_generator.cpp
)

set(HEADER_PROJECT_FILES
  src/include/gates_path_generator.hpp
)

# add_library(${PROJECT_NAME}
#   ${CPP_PROJECT_FILES}
#   ${HEADER_PROJECT_FILES}
# )
# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/source/gates_path_generator_node.cpp  ${CPP_PROJECT_FILES} ${HEADER_PROJECT_FILES}) 
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})