cmake_minimum_required(VERSION 3.8)
project(video_recorder)

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
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rviz_cinematographer_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)

add_library(${PROJECT_NAME} SHARED
  src/video_recorder.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${sensor_msgs_TARGETS}
  ${rviz_cinematographer_msgs_TARGETS}
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "video_recorder::VideoRecorder"
  EXECUTABLE video_recorder_node
)

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
  DIRECTORY launch watermark
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  sensor_msgs
  rviz_cinematographer_msgs
  OpenCV
)

ament_package()
