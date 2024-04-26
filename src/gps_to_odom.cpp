#include "ros/ros.h"
#include <cmath>
#include "Eigen/Dense"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;
using namespace Eigen;

class Gps_To_Odom {

	private:
		ros::Publisher pub; // Publisher;
		ros::Subscriber sub; // Subscriber;
		ros::NodeHandle nh; // Node handler;
		
		double lat_r, lon_r, alt_r; // Variables for parameters from launch file;
		Vector3d coordECEF_r;
		Vector3d last_coords, last_heading; // Saved variables of last message received;
		double a = 6378137.0, b = 6356752.0; // Earth constant;
		bool first_message_received = false;

		double epsilon = 0.05; // Epsilon value for the computation of the heading;

		// Degrees to radians function;
		double radians(double degrees) { return degrees * M_PI / 180.0; }
		
		// Compute E square constant;
		double computeESquare() { return 1 - (pow(b, 2) / pow(a, 2)); }
		
		// Comput N(phi) value;
		double computeN(double phi, double eSquare) { return a / sqrt(1 - eSquare * pow(sin(radians(phi)), 2)); }

		// Rotate a 3d vector around the z-axis;
		Vector3d rotateVector(Vector3d vector) {
			double tetha = radians(128.5); // APPROXIMATVEEE, THE VALUES ARE TOO NOISY TO RETRIEVE A SPECIFIC ANGLE TO ROTATE THE VECTOR WITH...;
			Matrix3d rotation;
			rotation << cos(tetha), -sin(tetha), 0,
						sin(tetha), cos(tetha), 0,
						0, 0, 1;
			return rotation * vector;
		}
		
		// Compute the heading of the robot with the current coordinates and the last set of coordinates received;
		Vector3d computeHeading(Vector3d coords, Vector3d last_coords) {
			Vector3d normV = (coords - last_coords).normalized();
			return normV;
		}

		// Compute heading vector to quaternion form;
		tf::Quaternion computeRotation(Vector3d heading) {
			tf::Quaternion rotation;
			double yaw = atan2(heading.y(), heading.x()); // Compute heading vector rotation on 2d xy-plane;

			rotation.setRPY(0, 0, yaw); // Set the quaternion rotation with the yaw angle;
			rotation.normalize(); // Normalize the quaternion (don't know if it is necessary);

			return rotation;
		}
	
	public:
		
		// Constructor;
		Gps_To_Odom() {
			sub = nh.subscribe("/fix", 100, &Gps_To_Odom::gpsConvertCallback, this); // Initialize the subscriber;
			pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 100); // Initialize the publisher;
			
			// Get initial variables from ROS parameter server, those set in the launch file;
			nh.getParam("gps_to_odom/lat_r", lat_r);
			nh.getParam("gps_to_odom/lon_r", lon_r);
			nh.getParam("gps_to_odom/alt_r", alt_r);
			
			Vector3d refPoint(lat_r, lon_r, alt_r);
			coordECEF_r = convertGPStoECEF(refPoint);
		}
		
		// GPS coordinates system to ECEF coordinates system;
		Vector3d convertGPStoECEF(Vector3d coordGPS) {
			double lat = coordGPS(0), lon = coordGPS(1), alt = coordGPS(2);
			double eSquare = computeESquare();
			double NofPHI = computeN(lat, eSquare);
			double lar_radians = radians(lat), lon_radians = radians(lon);

			double x = (NofPHI + alt) * cos(lar_radians) * cos(lon_radians);
			double y = (NofPHI + alt) * cos(lar_radians) * sin(lon_radians);
			double z = (NofPHI * (1 - eSquare) + alt) * sin(lar_radians);
			
			Vector3d coordECEF(x, y, z);
			return coordECEF;
		}
		
		// ECEF coordinates system to ENU coordinates system, using the coordECEF_r vector as reference;
		Vector3d convertECEFtoENU(Vector3d coordECEF) {
			double rad_lat_r = radians(lat_r), rad_lon_r = radians(lon_r);
			Matrix3d mat;

			mat << -sin(rad_lon_r), cos(rad_lon_r), 0,
				   -sin(rad_lat_r) * cos(rad_lon_r), -sin(rad_lat_r) * sin(rad_lon_r), cos(rad_lat_r),
				   cos(rad_lat_r) * cos(rad_lon_r),  cos(rad_lat_r) * sin(rad_lon_r),  sin(rad_lat_r);

			return mat * (coordECEF - coordECEF_r);
		}
		
		// Callback of the subscriber to /fix gps messages;
		void gpsConvertCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
			// Get msg parameters that we need;
			double lat = msg->latitude;
			double lon = msg->longitude;
			double alt = msg->altitude;
			
			Vector3d coords(lat, lon, alt);
			coords = convertGPStoECEF(coords);
			coords = convertECEFtoENU(coords);
			coords = rotateVector(coords);
			
			nav_msgs::Odometry odom_msg;
			odom_msg.header.stamp = ros::Time::now(); // Set current time of message;
			
			// Set position in the message;
			odom_msg.pose.pose.position.x = coords(0);
			odom_msg.pose.pose.position.y = coords(1);
			odom_msg.pose.pose.position.z = coords(2);
			
			Vector3d heading;
			tf::Quaternion rotation;

			if (!first_message_received) {
				rotation.setRPY(0.0, 0.0, 0.0);  // Set the initial robot rotation;
				heading << 0.0, 0.0, 0.0;  // Set the initial robot heading;
				first_message_received = true; // I received at least one message;
			}
			else {
				if ((coords - last_coords).norm() <= epsilon) { heading = last_heading; } // If the robot has not moved at least epsilon from last time then don't recompute heading;
				else {
					heading = computeHeading(coords, last_coords);
					last_heading = heading;
				}
				// ROS_INFO("Heading: %f, %f, %f\n", heading(0), heading(1), heading(2));

				rotation = computeRotation(heading);
			}
			tf::quaternionTFToMsg(rotation, odom_msg.pose.pose.orientation); // Transform the TF quaternion we created to geometry_msgs which is the type required by the odometry message we want to pub;

			last_coords = coords; // Save this coordinates as last_coords;
			
			pub.publish(odom_msg); // Publish our message;
			ROS_INFO("NODE1: Pub odom => pose:[x: %f, y: %f, z: %f]; orientation:[x: %f, y: %f, z: %f, w: %f]\n",
				odom_msg.pose.pose.position.x,
				odom_msg.pose.pose.position.y,
				odom_msg.pose.pose.position.z,

				odom_msg.pose.pose.orientation.x,
				odom_msg.pose.pose.orientation.y,
				odom_msg.pose.pose.orientation.z,
				odom_msg.pose.pose.orientation.w
			);
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "gps_to_odom"); // Init ROS;
	Gps_To_Odom gps_to_odom; // Call class constructor;
	ros::spin(); // Call spin;
	return 0;
}