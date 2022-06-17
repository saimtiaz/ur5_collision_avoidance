# ur5_collision_avoidance

## Introduction

End-to-end collision avoidance project for the UR5. Utilizes CollisionIK and a RealSense D435 Camera to detect and avoid obstacles. This project includes everything from calibration, point cloud processing, object detection, UR5 movement, and collision avoidance.


## Installation Instructions

## General Tasks

### UR5 IP Address

TODO: Make this information more clear by actually checking this on the robot
The UR5 IP Address can be obtained by starting up the UR5 from the control panel and navigating to the about information

### Turning on the camera

1. Navigate and source to ur5_ws:
    1. Run the following command to begin publishing camera data:
        ```bash
        roslaunch realsense2_camera rs_rgbd.launch
        ```

## Calibration

## Point Cloud Scan

## Point Cloud Cluster Extraction

## Cluster to Collision Object Extraction

## Robot Movement with CollisionIK

1. Navigate to catkin_ws/src/relaxed_ik_ros1/relaxed_ik_core/config/info_files/ur5_info.yaml
1. Make sure 'starting_config' is set the same as 'STARTING_CONFIG' in ur5_ws/src/mover/src/mover.py 

1. Navigate and source to ur5_ws:
    1. Launch the movement script:
        ```bash
        rosrun mover mover.py
        ```
1. Navigate and source to catkin_ws:
    1. View the robot arm:
        ```bash
        roslaunch relaxed_ik_ros1 rviz_viewer.launch
        ```
    1. Launch the CollisionIK solver:
        ```bash
        roslaunch relaxed_ik_ros1 relaxed_ik_rust.launch
        ```
    1. Start the solver:
        ```bash
        rosparam set /simulation_time go
        ```
    1. Launch the keyboard control:
        ```bash
        rosrun relaxed_ik_ros1 keyboard_ikgoal_driver.py
        ```


1. To use the keyboard controller, please ensure that the termainal window where <keyboard_ikgoal_driver.py> was run from has focus (i.e., make sure it's clicked), then use the following keystrokes:
```bash
    c - kill the controller controller script
    w - move chain 1 along +X
    x - move chain 1 along -X
    a - move chain 1 along +Y
    d - move chain 1 along -Y
    q - move chain 1 along +Z
    z - move chain 1 along -Z
    1 - rotate chain 1 around +X
    2 - rotate chain 1 around -X
    3 - rotate chain 1 around +Y
    4 - rotate chain 1 around -Y
    5 - rotate chain 1 around +Z
    6 rotate chain 1 around -Z

    i - move chain 2 along +X
    m - move chain 2 along -X
    j - move chain 2 along +Y
    l - move chain 2 along -Y
    u - move chain 2 along +Z
    n - move chain 2 along -Z
    = - rotate chain 2 around +X
    - - rotate chain 2 around -X
    0 - rotate chain 2 around +Y
    9 - rotate chain 2 around -Y
    8 - rotate chain 2 around +Z
    7 - rotate chain 2 around -Z
```

## Existing Issues

## Future Directions

