# ur5_collision_avoidance

## Introduction

End-to-end collision avoidance project for the UR5. Utilizes CollisionIK and a RealSense D435 Camera to detect and avoid obstacles. This project includes everything from calibration, point cloud processing, object detection, UR5 movement, and collision avoidance.

### Use Case
This package can be used for the following:
1. Calibrate a RealSense camera to the UR5 when attached to the end-effector
2. Create a point cloud scan of a working environment given a list of scan positions
3. Extract collision objects for use by CollisionIK using multi-level clustering techniques:
    1. K-Means
    2. Mini-Batch K-Means
    3. DBSCAN
    4. Any combination of the following above
4. Combine all of the above to enable the UR5 to avoid obstacles during real-time teleoperation for any working environment and teleoperation use case
5. Collision avoidance is meant for static obstacles only currently, but future work may enable dynamic collision avoidance as well.

### Demo
![scan](https://github.com/saimtiaz/ur5_collision_avoidance/blob/main/Figures/setup/scanPic.jpeg)

Scanning the working environment with the UR5 and RealSense camera

![raw point cloud](https://github.com/saimtiaz/ur5_collision_avoidance/blob/main/Figures/pointCloud/pointCloudDBview.png)

The raw point cloud from the scan after voxel grid and range filtering

![alt](https://github.com/saimtiaz/ur5_collision_avoidance/blob/main/Figures/miniBatch/minimeans33.png)

The clusters created from using DBSCAN followed by Mini-Batch K-Means Clustering with a k of 33

![DBSCAN + Mini-Batch K-Means k=33](https://github.com/saimtiaz/ur5_collision_avoidance/blob/main/Figures/miniBatch/miniBox33.png)

The axis-aligned bounding boxes created from the clusters from DBSCAN and Mini-Batch K-Means

![K-means k=99](https://github.com/saimtiaz/ur5_collision_avoidance/blob/main/Figures/kmeans/kbox99.png)

The axis-aligned bounding boxes created from using K-Means with a k of 99, resulting in the same number of bounding boxes. It creates a tighter fit of the environment but is significantly slower.


### Paper
The report: https://github.com/saimtiaz/ur5_collision_avoidance/blob/main/Report/End_to_End_Real_Time_Collision_Avoidance_Using_Clustering_Algorithms%20(6).pdf

The annotated bibliography: https://github.com/saimtiaz/ur5_collision_avoidance/blob/main/Report/Annotated_Bibliography.pdf

## Installation Instructions
1. Requirements
    1. Ubuntu 18.04
    2. ROS Melodic Morenia
2. General Installation
    1. Git clone the package
3. CollisionIK
    1. Follow the instructions at https://github.com/uwgraphics/relaxed_ik_ros1
4. UR Robot Driver
    1. Follow the instructions at https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
5. Calibration
    1. Install RealSense: https://github.com/IntelRealSense/realsense-ros
    2. Install MoveIt! for Ubuntu 18 : https://moveit.ros.org/install/
    3. Install the calibration software: https://github.com/portgasray/ur5_realsense_calibration

(Please note these instructions need to be tested but this should cover most of what is needed)

## General Tasks

### UR5 IP Address

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
1. Replace the first 7 numerical arguments with the 7-element matrix in the following order:
	1. x y z qx qy qz qw

## Point Cloud Scan
This step creates a point cloud scan based on the calibration from the prior step and a list of scan positions.
This step will result in a collection of point cloud files, which will then need to be placed into a folder and properly pointed to by cluster2obj.py in the cluster package.

1. Navigate and source to ur5_ws:
    1. Launch the ur5 robot driver:
        ```bash
	    roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=<robot_ip>
        ```
    1. Launch the realsense camera:
        ```bash
        roslaunch realsense2_camera rs_rgbd.launch
        ```
2. Navigate and source to catkin_ws
3. Navigate to the mover package:
    1. Modify the "POSITIONS" variable in scanMover.py to the desired scanning positions
    2. Modify the "HOST" and "PORT" variable to match up with the UR5 configuration
    3. Launch the "scanMover.py" script:
        ```bash
	    rosrun mover scanMover.py
        ```
4. Navigate to the pclTransform package:
    1. Launch the point cloud transform:
        ```bash
	    roslaunch pclTransform cloudTransform.launch
        ```
5. Launch the pcdCreate package:
    ```bash
        rosrun pcdCreate pcdCreator
    ```
    (NOTE: The pcdCreator.cpp file setLeafSize hyperparameter is useful to modify depending on the fidelity needed for the point cloud.)
6. Return back to the scanMover terminal window. The provided keyboard controller has the following commands:
```bash
    c - Close the point cloud scan
    n - Move to the next point cloud scan position
    p - Capture the next point cloud scan frame
```
(Please wait a short amount of time between moving to the next position and capturing a scan to allow for accurate point cloud scanning.)

## Point Cloud Cluster Extraction
This section will utilize Euclidean clustering to decompose the point cloud into clusters and planar objects.

This is optional as k-means clustering should be sufficient, and because euclidean clustering is already included in the cluster package in the following step.

This code is incomplete as it can only process one point cloud file at a time and not the entire scan.


1. Navigate and source to catkin_ws:
    1. Modify "capture1.pcd" to the correct point cloud target file
    1. Launch the euclidean cluster extraction script:
        ```bash
        rosrun cluster cluster_extraction
        ```
1. This will create a point cloud file for each extracted cluster. Move these files into a folder and remember to have the cluster2obj.py file point to this folder. 

## Cluster to Collision Object Extraction
This section will convert point cloud data into collision objects that are usable by CollisionIK
This code will automatically populate CollisionIK with the collision objects using k-means clustering


1. Navigate and source to catkin_ws:
2. Move all the point cloud scan files into a folder.
3. Open the cluster/cluster2obj.py script:
	1. Modify the 'is_windows' parameter to true or false based on operating system.
	2. Modify the 'point_cloud_radius' parameter based on the desired size of the operating environment. The UR5 has an operating length of 0.85 meters
	3. Modify the 'p_val' parameter based on the desired percent of the point cloud data to be excluded. 
	4. Modify the "_param" variables to the desired hyperparameters.
	5. Modify the "listOfParamLists" to the different combinations of clustering desired. 
	
	For example:
	
	listOfParamLists = [d, m] 
	
	This will first run DBSCAN on the point cloud, and then Mini-Batch K-means clustering on each of the DBSCAN extracted clusters.
	
4. Launch the cluster to collision object script:
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
1. The file directory system where point clouds are stored needs to be cleaned
2. A lot of the steps are not currently interconnected so there is a lot of manual file transfering involved
3. The ROS topics may not line up due to issues of getting the camera to work 

## Future Directions
1. Hyperparameter Exploration: Explore how different choices of hyperparameters and clustering combinations affect object extraction performance and runtime speed for different environments. Most of the hyperparameter exploration was only done on one situation, a cluttered lab table next to a wall.
2. Mesh filtering: Extending to dynamic obstacles may be possible by having an initilization step create a mesh filter around static obstacles and then detect dynamic obstacles outside of the mesh filter.
3. Oriented Bounding Boxes (OBBs): OBBs could be used instead of axis-aligned bounding boxes (AABBs) to produce a tighter fit around objects and produce a speedup on dynamic obstacles due to rotational invariance.
4. Occupancy Gridding: Occupancy grids with ray casting techniques would provide high surface fidelity as well as potentially update fast enough for real-time use. However, work needs to be done to explore how CollisionIK can be integrated with an occupancy grid.

