cmake_minimum_required(VERSION 2.8.3)
project(transform_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  rqt_gui
  rqt_gui_cpp
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(transform_manager_plugin_SRCS
    src/transform_manager_plugin.cpp
)

set(transform_manager_plugin_HDRS
    src/transform_manager_plugin.h
)

set(transform_manager_plugin_UIS
    src/transform_manager_plugin.ui
)

set(transform_manager_INCLUDE_DIRECTORIES
  
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_package(
    INCLUDE_DIRS ${transform_manager_INCLUDE_DIRECTORIES}
    LIBRARIES transform_manager_plugin
    CATKIN_DEPENDS roscpp tf2 tf2_ros rqt_gui_cpp rqt_gui
#  DEPENDS system_lib
)

set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
qt5_wrap_ui(transform_manager_plugin_UIS_H ${transform_manager_plugin_UIS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(
    ${transform_manager_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
)

add_library(transform_manager_plugin
    ${transform_manager_plugin_SRCS}
    ${transform_manager_plugin_MOCS}
    ${transform_manager_plugin_UIS_H}
)

qt5_use_modules(transform_manager_plugin Widgets)

target_link_libraries(transform_manager_plugin
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(transform_manager_plugin)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS transform_manager_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
