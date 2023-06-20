# PCL TASK

## Task Description

Obstacle Avoidance using Pointcloud data from the kinect on the 4 wheel bot integrating it with GPS based Autonomous traversal. Obstacles visualisation on RViz.

## PCL Overview

The code subscribes to the /kinect/depth/points topic, which provides the input point cloud data from a Kinect sensor. The cloudCallback function is called whenever a new point cloud message is received.

The main steps of the algorithm are as follows:

    Convert the received sensor_msgs::PointCloud2 message to a pcl::PointCloud<pcl::PointXYZRGB> point cloud format.
    Preprocess the point cloud by filtering out points outside a specified range.
    Perform ground plane elimination using RANSAC (Random Sample Consensus) to separate the ground plane from the rest of the objects.
    Extract the objects (non-ground points) from the filtered cloud.
    Perform cluster extraction to group nearby points into separate clusters based on their spatial proximity.
    Color each segmented cluster object differently for visualization purposes.
    Convert the colored point cloud back to sensor_msgs::PointCloud2 format.
    Publish the segmented and colored point cloud on the output_cloud topic.
    
 ## Gazebo and RVIZ Visualisation
    
![Screenshot from 2023-06-10 16-44-41](https://github.com/MRM-AIA-TP-2024/MRM_VyshnavDN/assets/96124935/e14ec095-5bca-4a77-9384-867779e861d5)

![Screenshot from 2023-06-10 16-53-58](https://github.com/MRM-AIA-TP-2024/MRM_VyshnavDN/assets/96124935/967b24d4-857d-48b3-bb34-4214fb16bfaf)

## RQT Graph
![Screenshot from 2023-06-11 01-33-00](https://github.com/MRM-AIA-TP-2024/MRM_VyshnavDN/assets/96124935/29d0ebff-2228-46af-967a-63a9f471b1cc)

## Navigation overview

Point Cloud data from kinect is converted to laserscan and that laserscan data is used for obstacle avoidance. Point to Point traversal using GPS Coordinates.

### Ros topics and Messages used

For the PCL node:

    Subscribed rostopic: "/kinect/depth/points"
        ROS message type: sensor_msgs/PointCloud2
        Description: Input point cloud topic from the Kinect sensor.

    Published rostopic: "output_cloud"
        ROS message type: sensor_msgs/PointCloud2
        Description: Point cloud with segmented and colored clusters.

For the Navigation node:

    Subscribed rostopic: "/gps/fix"
        ROS message type: sensor_msgs/NavSatFix
        Description: GPS fix information.

    Subscribed rostopic: "/imu"
        ROS message type: sensor_msgs/Imu
        Description: IMU (Inertial Measurement Unit) data.

    Subscribed rostopic: "/pctolaser/scan"
        ROS message type: sensor_msgs/LaserScan
        Description: Laser scan data.

    Subscribed rostopic: "/kinect/depth/points"
        ROS message type: sensor_msgs/PointCloud2
        Description: Input point cloud topic from the Kinect sensor.

    Published rostopic: "/filtered_cloud"
        ROS message type: sensor_msgs/PointCloud2
        Description: Filtered point cloud after eliminating the ground plane.

    Published rostopic: "/cmd_vel"
        ROS message type: geometry_msgs/Twist
        Description: Velocity command for controlling the robot's movement.
        
## Link to YouTube Video

https://www.youtube.com/watch?v=3ADccpJb30E&ab_channel=VyshnavDavanagere

https://www.youtube.com/watch?v=gRmwRVPQdLI&ab_channel=VyshnavDavanagere
