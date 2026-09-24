# Rviz2 Cinematographer

An rqt plugin to create and edit trajectories for the rviz2 camera and record its views in a video.

This is a ROS 2 port of [AIS-Bonn/rviz_cinematographer](https://github.com/AIS-Bonn/rviz_cinematographer),
released under the new package names `rviz2_cinematographer_*`.

This branch targets **ROS 2** (developed against Jazzy, should work with Humble and newer).
The packages are built with `ament_cmake`/`colcon`, the view controller is an `rviz2` plugin,
the GUI is an `rqt_gui_cpp` plugin and the video recorder is a composable `rclcpp` node.

# Packages

| Package | Description |
|---|---|
| [rviz2_cinematographer_msgs](rviz2_cinematographer_msgs) | Message definitions |
| [rviz2_cinematographer_view_controller](rviz2_cinematographer_view_controller) | rviz2 view controller plugin moving the camera along trajectories |
| [rviz2_cinematographer_video_recorder](rviz2_cinematographer_video_recorder) | Composable node writing the rendered views to a video |
| [rviz2_cinematographer_gui](rviz2_cinematographer_gui) | rqt plugin to create and edit camera trajectories |

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
$ ros2 launch rviz2_cinematographer_gui rviz2_cinematographer_gui.launch.py
```

An example trajectory generated in about 3 minutes:

![Example](readme/output.gif)

Visualized [Model Data](https://grabcad.com/library/office-building-9).

# Further information

- [Instructions](rviz2_cinematographer_gui)
- [Details - Package Structure](readme)
- [Details - Rviz View Controller](rviz2_cinematographer_view_controller)
- [Details - Video Recorder](rviz2_cinematographer_video_recorder)
 
# Remark

The recorded video will contain a watermark in the bottom right corner.  
Feel free to deactivate it in the GUI.  
If you do so, please mention the *Rviz2 Cinematographer* in a comment somewhere around your video.  
Your viewers might also be interested in using this tool.

# License

Rviz2 Cinematographer is licensed under BSD-3.  
This repository includes an adapted version of the [rviz_animated_view_controller](https://github.com/UTNuclearRoboticsPublic/rviz_animated_view_controller) package which is a modification of the official ros [rviz_animated_view_controller](https://github.com/ros-visualization/rviz_animated_view_controller) package for ros kinetic.  
Both of the ladder are licensed under BSD-2.

# Special Thanks

This repository is a fork of [AIS-Bonn/rviz_cinematographer](https://github.com/AIS-Bonn/rviz_cinematographer)
by Jan Razlaw (Autonomous Intelligent Systems group, University of Bonn).
Many thanks to the original authors for creating the Rviz Cinematographer and releasing it as open source.
