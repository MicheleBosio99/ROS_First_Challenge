#include "ros/ros.h"
#include <cmath> // Or does it need ""?
#include "Eigen/Dense"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

using namespace std;
using namespace Eigen;

// Publisher object;
ros::Publisher pub;

// Declare global parameters;
double lat_r, lon_r, alt_r;
double last_lat, last_lon, last_heading;
double a = 6378137.0, b = 6356752.0;
bool first_message_received = false;

// Functions

double radians(double degrees) { return degrees * M_PI / 180.0; }

double computeE() { return 1 - (pow(b, 2) / pow(a, 2)); }

double computeN(double phi, double e) { return a / sqrt(1 - pow(e, 2) * pow(sin(radians(phi)), 2)); }

Vector3d convertGPStoECEF(Vector3d coordGPS) {
	double lat = coordGPS(0), lon = coordGPS(1), alt = coordGPS(2);
	double e = computeE();
	double NofPHI = computeN(lat, e);
	double x = (NofPHI + alt) * cos(radians(lat)) * cos(radians(lon));
	double y = (NofPHI + alt) * cos(radians(lat)) * sin(radians(lon));
	double z = (NofPHI * (1 - pow(e, 2)) + alt) * sin(radians(lat));
	
	Vector3d coordECEF(x, y, z);
	return coordECEF;
}

Vector3d convertECEFtoENU(Vector3d coordECEF) {
	Vector3d refPoint(lat_r, lon_r, alt_r);
	Vector3d coords = coordECEF - refPoint;
	
	double rad_lat_r = radians(lat_r), rad_lon_r = radians(lon_r);
	
	Matrix3d mat;
	mat << -sin(rad_lon_r), cos(rad_lon_r), 0,
	    -sin(rad_lat_r) * cos(rad_lon_r), -sin(rad_lat_r) * sin(rad_lon_r), cos(rad_lat_r),
	    cos(rad_lat_r) * cos(rad_lon_r),  cos(rad_lat_r) * sin(rad_lon_r),  sin(rad_lat_r);
	
	Vector3d coordENU = mat * coordECEF;
	return coordENU;
}

double computeHeading(double lat1, double lon1, double lat2, double lon2) {
	double d_lon = radians(lon2 - lon1);
	double y = sin(d_lon) * cos(radians(lat2));
	double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(d_lon);
	
	return atan2(y, x);
}

void convertCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	double lat = msg->latitude;
	double lon = msg->longitude;
	double alt = msg->altitude;
	
	Vector3d coords(lat, lon, alt);
	coords = convertGPStoECEF(coords);
	coords = convertECEFtoENU(coords);
	
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = ros::Time::now();
	
	odom_msg.pose.pose.position.x = coords(0);
	odom_msg.pose.pose.position.y = coords(1);
	odom_msg.pose.pose.position.z = coords(2);
	
	if (!first_message_received) {
		// Identity Quaternion as first orientation;
		odom_msg.pose.pose.orientation.w = 1.0;

	} else {
		double heading;
		// Has the robot moved since last message received?
		if (last_lat == lat && last_lon == lon) { heading = last_heading; }
		else {
			heading = computeHeading(last_lat, last_lon, lat, lon);
			last_heading = heading;
		}
		
		Quaterniond quat(AngleAxisd(heading, Vector3d::UnitZ()));
		odom_msg.pose.pose.orientation.x = quat.x();
		odom_msg.pose.pose.orientation.y = quat.y();
		odom_msg.pose.pose.orientation.z = quat.z();
		odom_msg.pose.pose.orientation.w = quat.w();
	}
	
	last_lat = lat;
	last_lon = lon;
	first_message_received = true;
	
	pub.publish(odom_msg);
	ROS_INFO("Publishing: [x: %f, y: %f, z: %f]\n", coords(0), coords(1), coords(2));
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "gps_to_odom");
	
	// Declare node handler;
	ros::NodeHandle nh;
	// Declare subscribtion to /fix topic;
	ros::Subscriber sub = nh.subscribe("/fix", 1, convertCallback);
	// Let ROS know you will publish /gps_odom messages from this node;
	pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 1);
	
	// Retrieve the initial parameters setted in the launch file;
	nh.getParam("lat_r", lat_r);
	nh.getParam("lon_r", lon_r);
	nh.getParam("alt_r", alt_r);
	
	ros::spin();
	return 0;
}
