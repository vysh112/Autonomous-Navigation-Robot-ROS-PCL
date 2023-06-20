#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

sensor_msgs::NavSatFix gpsrn;
sensor_msgs::Imu imurn;

double destlat, destlong, requiredheading;
double angularerror;

ros::Subscriber gpssub, imusub;
ros::Publisher cmd_vel;

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


/* double calcvelocity(double angularerror) {
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
} */

int main(int argc, char **argv) {

    ros::init(argc, argv, "att_node");
    ros::NodeHandle n;
    gpssub = n.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 100, gpscallback);
    imusub = n.subscribe<sensor_msgs::Imu>("/imu", 100, imucallback);
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
    double headingrn = tf::getYaw(imurn.orientation);
    double remainingdistance = calcdistance(gpsrn.latitude, gpsrn.longitude, destlat, destlong);
    angularerror= calcheading(headingrn);
    std::cout<<"ANGULAR ERROR IS "<<angularerror<<"\n";
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
    std::cout<<"Remaining distance: "<<remainingdistance<<"\n";
    ros::spinOnce();
    loop_rate.sleep();
    headingrn = tf::getYaw(imurn.orientation);
    remainingdistance = calcdistance(gpsrn.latitude, gpsrn.longitude, destlat, destlong);
    angularerror= calcheading(headingrn)-headingrn;
}

return 0;
}

