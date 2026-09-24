cmake_minimum_required(VERSION 3.10)
project(rviz2_cinematographer_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz2_cinematographer_msgs REQUIRED)
find_package(rviz2_cinematographer_video_recorder REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

# Build against the same Qt major version as qt_gui_cpp (Qt5 up to Kilted, Qt6 from
# Lyrical on). Lyrical exports qt_gui_cpp_USE_QT_MAJOR_VERSION, Rolling finds Qt6 in its
# CMake extras and older distros don't export any Qt information (always Qt5).
if(DEFINED qt_gui_cpp_USE_QT_MAJOR_VERSION)
  set(QT_VERSION_MAJOR ${qt_gui_cpp_USE_QT_MAJOR_VERSION})
elseif(Qt6_FOUND)
  set(QT_VERSION_MAJOR 6)
else()
  set(QT_VERSION_MAJOR 5)
endif()
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets)

set(UI_FILES
  src/rviz2_cinematographer_gui.ui
)

set(H_FILES
  include/rviz2_cinematographer_gui/rviz2_cinematographer_gui.h
)

set(RESOURCES
  src/resource.qrc
)

qt_wrap_ui(UIC_FILES ${UI_FILES})
qt_wrap_cpp(MOC_FILES ${H_FILES})
qt_add_resources(RES_SOURCES ${RESOURCES})

add_library(rviz2_cinematographer_gui_plugin SHARED
  ${UIC_FILES}
  ${MOC_FILES}
  ${RES_SOURCES}
  src/plugins.cpp
  src/rviz2_cinematographer_gui.cpp
)

# the plugin is loaded via pluginlib only - nothing links against it, so no interface is exported
target_include_directories(rviz2_cinematographer_gui_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(rviz2_cinematographer_gui_plugin PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  pluginlib::pluginlib
  ${qt_gui_cpp_TARGETS}
  ${rqt_gui_cpp_TARGETS}
  interactive_markers::interactive_markers
  tf2::tf2
  ${tf2_geometry_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${rviz2_cinematographer_msgs_TARGETS}
  rviz2_cinematographer_video_recorder::rviz2_cinematographer_video_recorder
  ${YAML_CPP_TARGET}
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Widgets
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

install(
  TARGETS rviz2_cinematographer_gui_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch trajectories icons
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
