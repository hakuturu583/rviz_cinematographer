# Rviz2 Cinematographer

[![CI](https://github.com/hakuturu583/rviz_cinematographer/actions/workflows/ci.yaml/badge.svg)](https://github.com/hakuturu583/rviz_cinematographer/actions/workflows/ci.yaml)

An rqt plugin to create and edit trajectories for the rviz2 camera and record its views in a video.

This is a ROS 2 port of [AIS-Bonn/rviz_cinematographer](https://github.com/AIS-Bonn/rviz_cinematographer),
released under the new package names `rviz2_cinematographer_*`.

Supported ROS 2 distributions: **Humble, Jazzy, Kilted, Lyrical and Rolling**, all from a single branch.
Distro differences (Qt5/Qt6, renamed headers) are handled with `__has_include` and CMake checks.
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
 
# Release

Releases are cut by [the release workflow](.github/workflows/release.yaml) (the same mechanism as
[simple_lanelet2](https://github.com/hakuturu583/simple_lanelet2)):

1. Put **exactly one** of the labels `release:major`, `release:minor` or `release:patch` on a PR.
   PRs without one are merged without a release.
2. When it is merged into `master`, the workflow bumps the version of every package
   (`tools/bump_version.py`), writes the `CHANGELOG.rst` sections from the commits since the
   last release, commits `chore(release): X.Y.Z` and pushes the tag `X.Y.Z`.
3. The tagged commit is built again on all supported distros, published as a GitHub release and
   released to the ROS buildfarm with `bloom-release`, one distro after another.

A release can also be started by hand from *Actions → Release → Run workflow* (bump level or an
exact version; `current` tags the version already in `package.xml`), or by pushing an `X.Y.Z` tag.
To hand-write changelog entries, add a `Forthcoming` section (e.g. with `catkin_generate_changelog`);
it is renamed to the new version instead of being generated.

### One-time bloom setup

bloom can only run unattended once the repository is known to the buildfarm:

1. Create the release repository (e.g. `hakuturu583/rviz2_cinematographer-release`) and a fork of
   [ros/rosdistro](https://github.com/ros/rosdistro) under the account that owns the token below.
2. Cut the first release (`0.2.0`: *Run workflow* with `current` and bloom unchecked), then run
   `bloom-release --rosdistro <distro> --track <distro> --new-track rviz2_cinematographer`
   interactively once per distro. This creates the tracks and the first rosdistro PRs.
3. Add the repository secret `BLOOM_GITHUB_TOKEN`: a classic personal access token with the
   `public_repo` and `workflow` scopes that can push to the release repository.

Optional repository variables: `BLOOM_ROSDISTROS` (space separated, default
`humble jazzy kilted lyrical rolling`), `BLOOM_REPOSITORY` (name in rosdistro, default
`rviz2_cinematographer`), `BLOOM_GITHUB_USER` (token owner, default the repository owner),
`BLOOM_GIT_NAME` / `BLOOM_GIT_EMAIL` (author of the release repository commits).
Without the secret the bloom step is skipped with a warning; tagging and GitHub releases still work.

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
