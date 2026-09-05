cmake_minimum_required(VERSION 3.8)
project(rviz_cinematographer_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Find includes in corresponding build directories (needed for ui_*.h)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_cinematographer_msgs REQUIRED)
find_package(video_recorder REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(UI_FILES
  src/rviz_cinematographer_gui.ui
)

set(H_FILES
  include/rviz_cinematographer_gui/rviz_cinematographer_gui.h
)

set(RESOURCES
  src/resource.qrc
)

qt5_wrap_ui(UIC_FILES ${UI_FILES})
qt5_wrap_cpp(MOC_FILES ${H_FILES})
qt5_add_resources(RES_SOURCES ${RESOURCES})

add_library(rviz_cinematographer_gui_plugin SHARED
  ${UIC_FILES}
  ${MOC_FILES}
  ${RES_SOURCES}
  src/plugins.cpp
  src/rviz_cinematographer_gui.cpp
)

target_include_directories(rviz_cinematographer_gui_plugin PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(rviz_cinematographer_gui_plugin PUBLIC
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  pluginlib::pluginlib
  ${qt_gui_cpp_TARGETS}
  ${rqt_gui_cpp_TARGETS}
  interactive_markers::interactive_markers
  tf2::tf2
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${rviz_cinematographer_msgs_TARGETS}
  video_recorder::video_recorder
  ${YAML_CPP_TARGET}
  Qt5::Core
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

install(
  TARGETS rviz_cinematographer_gui_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch trajectories icons
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  pluginlib
  qt_gui_cpp
  rqt_gui_cpp
  interactive_markers
  tf2
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  rviz_cinematographer_msgs
  video_recorder
)

ament_package()
