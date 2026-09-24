^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz2_cinematographer_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-24)
------------------
* Port to ROS 2 (ament_cmake / colcon, rviz2, rqt_gui_cpp, rclcpp components)
* Rename packages for release: ``rviz_cinematographer_*`` -> ``rviz2_cinematographer_*``
  and ``video_recorder`` -> ``rviz2_cinematographer_video_recorder``
  (include directories, C++ namespaces and plugin class names follow the new package names)
* Support Humble, Jazzy, Kilted, Lyrical and Rolling from a single branch (Qt5/Qt6, renamed headers
  and API changes handled with the preprocessor) and add GitHub Actions CI for all of them
* Contributors: Masaya Kataoka
