#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <algorithm>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher filtered_cloud_publisher;

ros::Subscriber point_cloud_subscriber;

//this the one yo

class obstacle_avoidance{
private:
sensor_msgs::NavSatFix gpsrn;
sensor_msgs::Imu imurn;
sensor_msgs::LaserScan lidarrn;
geometry_msgs::Twist velmsg;

double requiredheading;
double angularerror;
double remainingdistance, headingrn;
bool obstacleinheading=false;

ros::Subscriber gpssub, imusub;
ros::Publisher cmd_vel;
ros::Subscriber laser_subscriber;

float d =1.7;
float front = 5;
float frontright = 5;
float frontleft = 5;
float right = 5;
float left = 5;


public:
double destlat, destlong;
double last_lat = 40;
double last_long = 40;
obstacle_avoidance(ros::NodeHandle* n) {
        cmd_vel = n->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        gpssub = n->subscribe<sensor_msgs::NavSatFix>("/gps/fix", 1, &obstacle_avoidance::gpscallback, this);
        imusub = n->subscribe<sensor_msgs::Imu>("/imu", 1, &obstacle_avoidance::imucallback, this);
        laser_subscriber = n->subscribe<sensor_msgs::LaserScan>("/pctolaser/scan", 1, &obstacle_avoidance::lasercallback, this);
        point_cloud_subscriber = n->subscribe<sensor_msgs::PointCloud2>("/kinect/depth/points", 1, &obstacle_avoidance::pointcloudcallback, this);
        filtered_cloud_publisher = n->advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);

    }

    void gpscallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void imucallback(const sensor_msgs::Imu::ConstPtr& msg);
    double calcdistance(double lat1, double lon1, double lat2, double lon2);
    double calcheading(double headingrn);
    double calcvelocity(double angularerror);
    void lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void pointcloudcallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void takeaction();
    void gostraight();
    void turn();
    void eliminateGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

void obstacle_avoidance::eliminateGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    //filter out ground plane points
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    //z threshold for ground plane elimination
    float z_threshold = 0.9; 
    
    // Iterate through each point in the cloud
    for (pcl::PointXYZ point : cloud->points) {
        // Check if the point's z coordinate is above the threshold
        if (point.z > z_threshold) {
            // Add the point to the filtered cloud
            filtered_cloud->points.push_back(point);
        }
    }

    
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header.stamp = ros::Time::now();  // Assign the current time as the stamp
    filtered_cloud_msg.header.frame_id = cloud->header.frame_id;

   
    filtered_cloud_publisher.publish(filtered_cloud_msg);

    
}


void obstacle_avoidance::pointcloudcallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    eliminateGroundPlane(cloud);
}




void obstacle_avoidance::gpscallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    gpsrn = *msg;
    remainingdistance = calcdistance(gpsrn.latitude, gpsrn.longitude, destlat, destlong);
    }

void obstacle_avoidance::imucallback(const sensor_msgs::Imu::ConstPtr& msg){
    imurn = *msg;
    if(remainingdistance>1){
      if(front<d || frontright <d || frontleft<d){
        takeaction();
      }
      else if(right<1 || left<1){
        velmsg.linear.x=0.3;
        velmsg.angular.z=0;
        cmd_vel.publish(velmsg);
      }
      else{
        turn();
      }
    }

    else{
      velmsg.angular.z=0;
      cmd_vel.publish(velmsg);
      std::cout<<"Turnin done boi";
      gostraight();
    }
}

void obstacle_avoidance::lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){

    // Determine the number of samples in the scan message
  int num_samples = scan_msg->ranges.size();

  // Reset the range values in each region to a high value
  front=10;
  frontleft=10;
  frontright=10;
  left = 10;
  right = 10;

  // Check each sample in the scan message to see if it is within the threshold distance
  for (int i = 0; i < num_samples; i++) {
    double range = scan_msg->ranges[i];
    float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

    if (angle>=-0.5 && angle <= 0.5) {
      // Update the range value in the front region
     front = std::min(range, 10.0);
    } else if (angle>= 0.5 && angle <= 1.2) {
      // Update the range value in the front left region
      frontleft = std::min(range, 10.0);
    } else if (angle>=-1.2 && angle <= -0.5) {
      // Update the range value in the front right region
      frontright = std::min(range, 10.0);
    }
    else if (angle>=-1.75 && angle <= -1.35) {
      // Update the range value in the right region
      right = std::min(range, 10.0);
    }
    else if (angle>=1.35 && angle <= 1.75) {
      // Update the range value in the left region
      left = std::min(range, 10.0);
    }
  }

  if(front < 1) {
        ROS_WARN("Obstacle detected in front");
        velmsg.linear.x=0.0;
    velmsg.angular.z=0;
    cmd_vel.publish(velmsg);
      
        
        
    } else if (frontleft < 1) {
        ROS_WARN("Obstacle detected in front left");
        velmsg.linear.x=0.0;
    velmsg.angular.z=0;
    cmd_vel.publish(velmsg);
        
    } else if (frontright < 1) {
        ROS_WARN("Obstacle detected in front right");
        velmsg.linear.x=0.0;
    velmsg.angular.z=0;
    cmd_vel.publish(velmsg);
      
        
    }
}


double obstacle_avoidance::calcdistance(double lat1, double lon1, double lat2, double lon2){
    const double earth_radius = 6371000.0;
    double d_lat = (lat2 - lat1) * M_PI / 180.0;
    double d_lon = (lon2 - lon1) * M_PI / 180.0;
    double a = pow(sin(d_lat / 2), 2) + cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * pow(sin(d_lon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return earth_radius * c;
}

double obstacle_avoidance::calcheading(double headingrn){
    double requiredheading = atan2(destlat - gpsrn.latitude, destlong - gpsrn.longitude);
    double relativeheading = requiredheading - headingrn;
    if (relativeheading < -M_PI) {
        relativeheading += 2 * M_PI;
    } else if (relativeheading > M_PI) {
        relativeheading -= 2 * M_PI;
    }
    return relativeheading;
}

double obstacle_avoidance::calcvelocity(double angularerror){

    double k = 10; // tuning constant for proportional control
    double velocity = 0;
    if (std::abs(angularerror) > 0.1) {
        velocity = k * angularerror;
        if (angularerror < 0) {
            velocity = std::max(velocity, -0.1);
        } else {
            velocity = std::min(velocity, 0.1);
        }
    }
    return velocity;

}

void obstacle_avoidance::gostraight(){
  std::cout<<" /n Distance Remaining :"<<remainingdistance;
  if(remainingdistance>1){
    velmsg.linear.x=0.2;
    velmsg.angular.z=0;
    cmd_vel.publish(velmsg);
  }
  else{
    std::cout<<"REACHED";
    velmsg.linear.x=0.0;
    cmd_vel.publish(velmsg);

  }

}

void obstacle_avoidance::turn(){
headingrn = tf::getYaw(imurn.orientation);
remainingdistance = calcdistance(gpsrn.latitude, gpsrn.longitude, destlat, destlong);
angularerror= calcheading(headingrn);
  if(angularerror>0.07 || angularerror<-0.07)
    {   if(angularerror>0){
        velmsg.angular.z= 0.3*abs(angularerror);
        velmsg.linear.x=0.0;
        cmd_vel.publish(velmsg);}
        else{
        velmsg.angular.z= -0.3*abs(angularerror);
        velmsg.linear.x=0.0;
        cmd_vel.publish(velmsg);
        }
    }
    else{
        velmsg.angular.z=0;
        cmd_vel.publish(velmsg);
        std::cout<<"TURNING DONE";
        gostraight();
    }
    


}

void obstacle_avoidance::takeaction(){


if(frontleft < (d+0.5) &&  front> d && frontright > d ){
  std::cout<<" CASE 1 - Front left";
  velmsg.linear.x = 0.3;
  velmsg.angular.z = -0.7;
  cmd_vel.publish(velmsg);
}

else if(frontleft > d &&  front <(d+0.3)  && frontright > d ){
  std::cout<<" CASE 2 - Front";
  velmsg.linear.x = 0.3;
  //maximum angular velocity
double max_ang_vel = -0.7;

velmsg.angular.z = max_ang_vel ;

  cmd_vel.publish(velmsg);
}

else if(frontleft > d &&  front > d && frontright < (d+0.5) ){
  std::cout<<" CASE 3 - Front Right";
  velmsg.linear.x = 0.3;
  velmsg.angular.z = 0.7;
  cmd_vel.publish(velmsg);
}


else if(frontleft > d &&  front < d && frontright < d ){
  std::cout<<" CASE 4 - Front and Front Right";
  velmsg.linear.x = 0;
  velmsg.angular.z = 0.8;
  cmd_vel.publish(velmsg);
}

else if(frontleft < d &&  front > d && frontright < d ){
  std::cout<<" CASE 5 - Front left and front right";
  velmsg.linear.x = 0.3;
  velmsg.angular.z = 0;
  cmd_vel.publish(velmsg);
}

else if(frontleft < d &&  front < d && frontright > d ){
  std::cout<<" CASE 6 - Front and Front LEft";
  velmsg.linear.x = 0;
  velmsg.angular.z = -0.7;
  cmd_vel.publish(velmsg);
}

else if(frontleft < d &&  front<d && frontright < d ){
  std::cout<<" CASE 7 - Front and Front LEft and front right";
  velmsg.linear.x = 0;
  velmsg.angular.z = -0.7;
  cmd_vel.publish(velmsg);
}
else{
  std::cout<<"ELSE FINAL CASE";
        velmsg.angular.z=0;
        velmsg.linear.x=0.3;
        cmd_vel.publish(velmsg);
    }

}



int main(int argc, char **argv) {
ros::init(argc, argv, "attwithoa_node");
ros::NodeHandle n;
obstacle_avoidance oa = obstacle_avoidance(&n);
if (argc != 3) {
        ROS_ERROR("missing or additional args");
        return 1;
}
oa.destlat = atof(argv[1]);
oa.destlong = atof(argv[2]);
ros::Rate loop_rate(100);
    while(ros::ok()){
    
  
    

    
    ros::spinOnce();
    loop_rate.sleep();
    }


return 0;

}
