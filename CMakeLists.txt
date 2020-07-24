cmake_minimum_required(VERSION 2.8.3)
project(transform_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
)

catkin_package(
#    INCLUDE_DIRS include
#  LIBRARIES project11_transformations
    CATKIN_DEPENDS roscpp tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/transform_manager.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
