# General

Subscribes to images and starts to generate a video on receiving a *record*-message, optionally adding a watermark.

The recorder is a composable ROS 2 node (`rviz2_cinematographer_video_recorder::VideoRecorder`).
It is started automatically by the *rviz2_cinematographer_gui* plugin within the same process.
To run it standalone use

```
$ ros2 launch rviz2_cinematographer_video_recorder rviz2_cinematographer_video_recorder.launch.py
```
or
```
$ ros2 run rviz2_cinematographer_video_recorder video_recorder_node
```

# Messages

#### Inputs:  

1. **Topic** : /rviz/view_image  
   **Type** : sensor_msgs/msg/Image    
   **Purpose** : The image input.

2. **Topic** : /rviz/record  
   **Type** : rviz2_cinematographer_msgs/msg/Record  
   **Purpose** : Parameters for the output video + Starts recording.  
   Frame Rate, Codec, Output File Name, Add Watermark Flag.  

3. **Topic** : /rviz/finished_rendering_trajectory    
   **Type** : rviz2_cinematographer_msgs/msg/Finished    
   **Purpose** : Indicates that the input stream ended.  

#### Outputs:

1. **Topic** : /video_recorder/record_finished  
   **Type** : rviz2_cinematographer_msgs/msg/Finished  
   **Purpose** : Indicates that the input stream was fully processed.  

2. **Topic** : /video_recorder/wait_duration  
   **Type** : rviz2_cinematographer_msgs/msg/Wait  
   **Purpose** : The approximate time it takes to process most of the queue buffering the input images.    
   Is send if processing the images takes more time than generating and queueing.  
