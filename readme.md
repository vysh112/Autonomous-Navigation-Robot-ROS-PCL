
# Task Description:
## Autonomous Traversal of Four Wheeled Bot from one GPS coordinate to another GPS coordinate

A single roscpp traversal node is used wherein using the gps data, distance to end destination is calculated using haversine formula. IMU data is used for calculating the current orientation and then I calculated the required heading to rotate to. A proportional controller is implemented based on the angular difference and linear distance to destination coordinates

## Code Description

This code provides point to point GPS based navigation with obstacle avoidance functionality for the robot using GPS, IMU, and lidar sensor data.

The following dependencies are required:

    sensor_msgs/NavSatFix.h
    sensor_msgs/Imu.h
    sensor_msgs/LaserScan.h
    ros/ros.h
    std_msgs/String.h
    tf/transform_datatypes.h
    geometry_msgs/Twist.h
    cmath
    algorithm

### Class: obstacle_avoidance

This class represents the obstacle avoidance behavior for the robot. It contains private member variables and methods for handling sensor data, calculating distances and headings, and controlling the robot's movement.
Member Variables

    gpsrn: Sensor message for GPS data (sensor_msgs/NavSatFix)
    imurn: Sensor message for IMU data (sensor_msgs/Imu)
    lidarrn: Sensor message for laser scan data (sensor_msgs/LaserScan)
    velmsg: Twist message for controlling robot velocity (geometry_msgs/Twist)
    requiredheading: Required heading for the robot to reach the destination (double)
    angularerror: Angular error between the robot's current heading and the required heading (double)
    remainingdistance: Remaining distance to the destination (double)
    headingrn: Robot's current heading (double)
    obstacleinheading: Flag indicating if there is an obstacle in the robot's current heading (bool)
    gpssub: Subscriber for GPS data (ros::Subscriber)
    imusub: Subscriber for IMU data (ros::Subscriber)
    cmd_vel: Publisher for robot's velocity commands (ros::Publisher)
    laser_subscriber: Subscriber for laser scan data (ros::Subscriber)
    d: Distance threshold for obstacles (float)
    front: Range value in the front region from laser scan data (float)
    frontright: Range value in the front right region from laser scan data (float)
    frontleft: Range value in the front left region from laser scan data (float)
    right: Range value in the right region from laser scan data (float)
    left: Range value in the left region from laser scan data (float)
    destlat: Destination latitude (double)
    destlong: Destination longitude (double)
    last_lat: Last known latitude (double)
    last_long: Last known longitude (double)

### Methods

    obstacle_avoidance(ros::NodeHandle* n): Constructor for the obstacle_avoidance class. It initializes the ROS node, sets up subscribers and publishers, and assigns initial values to member variables.
    gpscallback(const sensor_msgs::NavSatFix::ConstPtr& msg): Callback function for GPS data. It updates the remainingdistance based on the current GPS position and the destination.
    imucallback(const sensor_msgs::Imu::ConstPtr& msg): Callback function for IMU data. It handles obstacle avoidance logic based on sensor data and controls the robot's movement.
    lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg): Callback function for laser scan data. It processes the scan data and identifies obstacles in different regions.
    calcdistance(double lat1, double lon1, double lat2, double lon2): Calculates the distance between two GPS coordinates using the Haversine formula.
    calcheading(double headingrn): Calculates the relative heading error between the robot's current heading and the required heading.
    calcvelocity(double angularerror): Calculates the angular velocity of the robot based on the angular error using proportional control.
    gostraight(): Moves the robot straight towards the destination while checking the remaining distance.
    turn(): Turns the robot towards the required heading while avoiding obstacles. It adjusts the angular velocity based on the angular error.
    takeaction(): Takes appropriate action based on the obstacle positions in different regions and sets the linear and angular velocities accordingly.
    main(int argc, char **argv): The main function of the program. It initializes the ROS node, creates an instance of the obstacle_avoidance class, sets the destination coordinates from command-line arguments, and starts the ROS spin loop.


## RQT Graph

![Screenshot from 2023-02-24 20-56-26](https://user-images.githubusercontent.com/96124935/221219255-bf6888cf-b80c-44fa-aaa8-742839b9716b.png)

## ROS Topics

![Screenshot from 2023-02-24 20-57-28](https://user-images.githubusercontent.com/96124935/221219338-1cfa2f70-4279-49ca-8f81-e6946bbe22df.png)

## ROS Messages

![Screenshot from 2023-02-27 19-25-44](https://user-images.githubusercontent.com/96124935/221584013-ab80217b-aeea-4eb3-940e-8d3fb9dc76ca.png)

![Screenshot from 2023-02-27 19-26-22](https://user-images.githubusercontent.com/96124935/221584019-5f41443d-181c-45f3-928f-2c46558d912b.png)
![Screenshot from 2023-02-27 19-27-46](https://user-images.githubusercontent.com/96124935/221584030-268cf7d3-cf51-442a-b158-e923248c188a.png)
![Screenshot from 2023-02-24 20-57-28](https://user-images.githubusercontent.com/96124935/221583944-17e260b8-9f6b-43c0-b527-d60ee4bb48ce.png)

## Link to YouTube Video for Navigation with Obstacle Avoidance Task:
https://www.youtube.com/watch?v=acHFoFqlU_A&ab_channel=VyshnavDavanagere

## PCL Task Video:
https://www.youtube.com/watch?v=3ADccpJb30E&ab_channel=VyshnavDavanagere

https://www.youtube.com/watch?v=gRmwRVPQdLI&ab_channel=VyshnavDavanagere
