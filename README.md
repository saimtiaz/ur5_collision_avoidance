# ur5_collision_avoidance

## Introduction

End-to-end collision avoidance project for the UR5. Utilizes CollisionIK and a RealSense D435 Camera to detect and avoid obstacles. This project includes everything from calibration, point cloud processing, object detection, UR5 movement, and collision avoidance.

### Use Case

### Demo

### Paper

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

1. The calibration code and instructions were adopted from the following link : https://github.com/portgasray/ur5_realsense_calibration

1. Take note of the IP address of the UR5

1. If you're calibrating the camera on the end of the robot, make sure eye_on_hand is set to true on ur5_realsense_handeyecalibration.launch
1. Else, it should be set to false

1. Plug in the realsense camera if not done so already

1. Navigate and source to ur5_ws:
    1. Launch the ur5 robot driver:
        ```bash
	    roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=<robot_ip>
        ```
    1. Launch the MoveIt Planner:
        ```bash
	    roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
        ```
    1. Launch the realsense camera:
        ```bash
        roslaunch realsense2_camera rs_rgbd.launch
        ```
    1. Launch the calibation code:
        ```bash
	    roslaunch easy_handeye ur5_realsense_handeyecalibration.launch
        ```

1. Look at the GUI titled "rqt_easy_handeye.perspective - rqt"
1. Navigate to "Plugins -> Visualization -> Image View" from the options at the top
1. Select "/aruco_tracker/result" in the topic listbox

1. Move the UR5 until the calibration plane is in place 
1. Click "Take Sample" under the "Actions" section
1. Move the UR5 to a different position. Take samples only if the three estimated axes look correct
1. Take at least 20 samples
1. Click compute to acquire the 7-element matrix (x, y, z, qx, qy, qz, qw)

1. Navigate to catkin_ws/src/pclTransform/launch/cloudTransform.launch
1. Locate the node named "calibrationTransform"
1. Replace the first 7 numerical arguments witht he 7-element matrix in the following order:
	1. x y z qx qy qz qw

## Point Cloud Scan
1. IN DEVELOPMENT

## Point Cloud Cluster Extraction
This section will utilize Euclidean clustering to decompose the point cloud into clusters and planar objects
This is optional as k-means clustering should be sufficient
Please note this instruction set is incomplete as there are some additional steps to get the euclidean clusters working with the next section.

1. Navigate and source to catkin_ws:
    1. Launch the euclidean cluster extraction script:
        ```bash
        rosrun cluster cluster_extraction
        ```

## Cluster to Collision Object Extraction
This section will convert point cloud data into collision objects that are usable by CollisionIK
This code will automatically populate CollisionIK with the collision objects using k-means clustering


1. Navigate and source to catkin_ws:
	1. Launch the cluster to collision object script:
        ```bash
    	rosrun cluster cluster2obj.py
        ```
    
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
1. Point Cloud Scan instructions are forthcoming
1. Point Cloud Cluster Extraction instructions are incomplete
1. The file directory system where point clouds are stored needs to be cleaned

## Future Directions

