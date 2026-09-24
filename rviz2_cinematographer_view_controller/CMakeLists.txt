cmake_minimum_required(VERSION 3.10)
project(rviz2_cinematographer_view_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rviz2_cinematographer_msgs REQUIRED)

# Qt
# Build against the same Qt major version as rviz_common
# (Qt5 up to Kilted, Qt6 from Lyrical on).
if(Qt6_FOUND)
  set(QT_VERSION_MAJOR 6)
elseif(Qt5_FOUND)
  set(QT_VERSION_MAJOR 5)
else()
  find_package(Qt6 QUIET COMPONENTS Core Widgets)
  if(Qt6_FOUND)
    set(QT_VERSION_MAJOR 6)
  else()
    set(QT_VERSION_MAJOR 5)
  endif()
endif()
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/rviz2_cinematographer_view_controller.h
  src/rviz2_cinematographer_view_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  pluginlib::pluginlib
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_ogre_vendor::OgreMain
  image_transport::image_transport
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${rviz2_cinematographer_msgs_TARGETS}
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Widgets
)

# rviz_common 15.1.8 (Lyrical) introduced ViewController::update(std::chrono::nanoseconds, ...)
# and rviz_common 16 (Rolling) removed update(float, float).
if(rviz_common_VERSION VERSION_GREATER_EQUAL "15.1.8")
  target_compile_definitions(${PROJECT_NAME} PUBLIC RVIZ2_CINEMATOGRAPHER_RVIZ_COMMON_CHRONO_UPDATE=1)
endif()

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  pluginlib
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  image_transport
  rviz2_cinematographer_msgs
)

ament_package()
