# Rviz Cinematographer

An rqt plugin to create and edit trajectories for the rviz camera and record its views in a video.

This branch targets **ROS 2** (developed against Jazzy, should work with Humble and newer).
The packages are built with `ament_cmake`/`colcon`, the view controller is an `rviz2` plugin,
the GUI is an `rqt_gui_cpp` plugin and the video recorder is a composable `rclcpp` node.

# Build

```
$ cd ~/ros2_ws/src
$ git clone <this repository>
$ cd ~/ros2_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install
$ source install/setup.bash
```

# Quick start

```
$ ros2 launch rviz_cinematographer_gui rviz_cinematographer_gui.launch.py
```

An example trajectory generated in about 3 minutes:

![Example](readme/output.gif)

Visualized [Model Data](https://grabcad.com/library/office-building-9).

# Further information

- [Instructions](rviz_cinematographer_gui)
- [Details - Package Structure](readme)
- [Details - Rviz View Controller](rviz_cinematographer_view_controller)
- [Details - Video Recorder](video_recorder)
 
# Remark

The recorded video will contain a watermark in the bottom right corner.  
Feel free to deactivate it in the GUI.  
If you do so, please mention the *Rviz Cinematographer* in a comment somewhere around your video.  
Your viewers might also be interested in using this tool.

# License

Rviz Cinematographer is licensed under BSD-3.  
This repository includes an adapted version of the [rviz_animated_view_controller](https://github.com/UTNuclearRoboticsPublic/rviz_animated_view_controller) package which is a modification of the official ros [rviz_animated_view_controller](https://github.com/ros-visualization/rviz_animated_view_controller) package for ros kinetic.  
Both of the ladder are licensed under BSD-2.
