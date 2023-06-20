#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <algorithm>

sensor_msgs::NavSatFix gpsrn;
sensor_msgs::Imu imurn;
sensor_msgs::LaserScan lidarrn;

double destlat, destlong, requiredheading;
double angularerror;
bool obstacleinheading=false;

ros::Subscriber gpssub, imusub;
ros::Publisher cmd_vel;
ros::Subscriber laser_subscriber;

float d;
float front = 5;
float frontright = 5;
float frontleft = 5;
float right = 5;
float left = 5;

geometry_msgs::Twist velmsg;

double calcdistance(double lat1, double lon1, double lat2, double lon2) {
    const double earth_radius = 6371000.0;
    double d_lat = (lat2 - lat1) * M_PI / 180.0;
    double d_lon = (lon2 - lon1) * M_PI / 180.0;
    double a = pow(sin(d_lat / 2), 2) + cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * pow(sin(d_lon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return earth_radius * c;
}

void gpscallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    gpsrn = *msg;
}

void imucallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imurn = *msg;
}

/*void lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    lidarrn= *msg;
}*/
double calcheading(double headingrn){
    double requiredheading = atan2(destlat - gpsrn.latitude, destlong - gpsrn.longitude);
    double relativeheading = requiredheading - headingrn;
    if (relativeheading < -M_PI) {
        relativeheading += 2 * M_PI;
    } else if (relativeheading > M_PI) {
        relativeheading -= 2 * M_PI;
    }
    return relativeheading;
}




 double calcvelocity(double angularerror) {
    double k = 10; // tuning constant for proportional control
    double velocity = 0;
    if (std::abs(angularerror) > 0.1) {
        velocity = k * angularerror;
        if (angularerror < 0) {
            velocity = std::max(velocity, -0.06);
        } else {
            velocity = std::min(velocity, 0.06);
        }
    }
    return velocity;
} 


void lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  
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
    } else if (angle>= 0.5 && angle <= 1.1) {
      // Update the range value in the front left region
      frontleft = std::min(range, 10.0);
    } else if (angle>=-1.1 && angle <= -0.5) {
      // Update the range value in the front right region
      frontright = std::min(range, 10.0);
    }
    else if (angle>=-1.5 && angle <= -1.1) {
      // Update the range value in the right region
      right = std::min(range, 10.0);
    }
    else if (angle>=1.1 && angle <= 1.5) {
      // Update the range value in the front right region
      left = std::min(range, 10.0);
    }
  }

  if(front < 1) {
        ROS_WARN("Obstacle detected in front");
    } else if (frontleft < 1) {
        ROS_WARN("Obstacle detected in front left");
        
    } else if (frontright < 1) {
        ROS_WARN("Obstacle detected in front right");
        
    }


    
   }

void takeaction(){
  double headingrn = tf::getYaw(imurn.orientation);
    double remainingdistance = calcdistance(gpsrn.latitude, gpsrn.longitude, destlat, destlong);
  angularerror= calcheading(headingrn);

d = 1.8;

if(frontleft < d &&  front> d && frontright > d ){
  std::cout<<" CASE 1 - Front left";
  velmsg.linear.x = 0;
  velmsg.angular.z = -0.3;
  cmd_vel.publish(velmsg);
}

else if(frontleft > d &&  front < d && frontright > d ){
  std::cout<<" CASE 2 - Front";
  velmsg.linear.x = 0;
  velmsg.angular.z = 0.3;
  cmd_vel.publish(velmsg);
}

else if(frontleft > d &&  front > d && frontright < d ){
  std::cout<<" CASE 3 - Front Right";
  velmsg.linear.x = 0;
  velmsg.angular.z = 0.3;
  cmd_vel.publish(velmsg);
}


else if(frontleft > d &&  front < d && frontright < d ){
  std::cout<<" CASE 4 - Front and Front Right";
  velmsg.linear.x = 0;
  velmsg.angular.z = 0.3;
  cmd_vel.publish(velmsg);
}

else if(frontleft < d &&  front > d && frontright < d ){
  std::cout<<" CASE 5 - Front left and front right";
  velmsg.linear.x = 0;
  velmsg.angular.z = 0.3;
  cmd_vel.publish(velmsg);
}

else if(frontleft < d &&  front < d && frontright > d ){
  std::cout<<" CASE 6 - Front and Front LEft";
  velmsg.linear.x = 0;
  velmsg.angular.z = -0.3;
  cmd_vel.publish(velmsg);
}

else if(frontleft < d &&  front < d && frontright < d ){
  std::cout<<" CASE 7 - Front and Front LEft and front right";
  velmsg.linear.x = 0;
  velmsg.angular.z = 0.3;
  cmd_vel.publish(velmsg);
}
else if(front>d && frontleft >d &&frontleft>d && right>d && left>d){
    if(remainingdistance<0.7){
        velmsg.linear.x=0.0;
        velmsg.angular.z=0;
        std::cout<<"REACHED";

    }
    else if(angularerror>0.07 || angularerror<-0.07)
    {   if(angularerror>0){
        velmsg.angular.z= 0.3*abs(angularerror);
        velmsg.linear.x=0.0;}
        else{
            
        velmsg.angular.z= -0.5*abs(angularerror);
        velmsg.linear.x=0.0;
        }
    }
    else{
        velmsg.angular.z=0;
        velmsg.linear.x=0.2*remainingdistance;
    }

    
    
    cmd_vel.publish(velmsg);
}
else{
        velmsg.angular.z=0;
        velmsg.linear.x=0.2;
        cmd_vel.publish(velmsg);
    }



}


int main(int argc, char **argv) {

    ros::init(argc, argv, "att_node");
    ros::NodeHandle n;
    gpssub = n.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 100, gpscallback);
    imusub = n.subscribe<sensor_msgs::Imu>("/imu", 100, imucallback);
    laser_subscriber = n.subscribe<sensor_msgs::LaserScan>("scan", 100, lasercallback);
    
    if (argc != 3) {
        ROS_ERROR("missing or additional args");
        return 1;
    }
    destlat = atof(argv[1]);
    destlong = atof(argv[2]);

    ROS_INFO("Destination coordinates: %f, %f", destlat, destlong);
    cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate loop_rate(20);
    while(ros::ok()){
    
    
    takeaction();
    
    
    
    
    

    
    ros::spinOnce();
    loop_rate.sleep();
    }


return 0;
}

