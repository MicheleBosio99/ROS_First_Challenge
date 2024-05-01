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
		
		double lat_r, lon_r, alt_r;
		Vector3d coordECEF_r;
		Vector3d last_coords, last_heading;
		double a = 6378137.0, b = 6356752.0;
		bool first_message_received = false;
		
		double radians(double degrees) { return degrees * M_PI / 180.0; }
		
		double computeE() { return 1 - (pow(b, 2) / pow(a, 2)); }
		
		double computeN(double phi, double e) { return a / sqrt(1 - e * pow(sin(radians(phi)), 2)); }
		
		Vector3d computeHeading(Vector3d coords, Vector3d last_coords) {
			Vector3d normV = (coords - last_coords).normalized();
			return normV;
		}

		tf::Quaternion computeRotation(Vector3d heading) {
			tf::Quaternion rotation;
			Vector2d head2D = heading.head<2>(), xPos(1.0, 0.0);
			double yaw = acos(head2D.dot(xPos)) / ((head2D.norm()) * (xPos.norm())); // Compute angle between heading vector and x positive semiaxis;

			// ROS_INFO("Angle: %f", yaw * 180.0 / M_PI);
			if(!isfinite(yaw)) {
				rotation.setRPY(0.0, 0.0, 0.0);
				return rotation;
			}

			// NB: the only angle considered is the one in the xy plane, since I noticed the heading.z does not change its value and it's always 0 (meaning the robot is not going up or down hill);
			rotation.setRPY(0, 0, yaw);
			rotation.normalized();
			return rotation;
		}
	
	public:
		
		Gps_To_Odom() {
			sub = nh.subscribe("/fix", 10, &Gps_To_Odom::gpsConvertCallback, this);
			pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 10);
			
			nh.getParam("gps_to_odom/lat_r", lat_r);
			nh.getParam("gps_to_odom/lon_r", lon_r);
			nh.getParam("gps_to_odom/alt_r", alt_r);
			
			Vector3d refPoint(lat_r, lon_r, alt_r);
			coordECEF_r = convertGPStoECEF(refPoint); // Convert the default parameters into ECEF coordinates;
		}
		
		Vector3d convertGPStoECEF(Vector3d coordGPS) {
			double lat = coordGPS(0), lon = coordGPS(1), alt = coordGPS(2);
			double e = computeE();
			double NofPHI = computeN(lat, e);
			double x = (NofPHI + alt) * cos(radians(lat)) * cos(radians(lon));
			double y = (NofPHI + alt) * cos(radians(lat)) * sin(radians(lon));
			double z = (NofPHI * (1 - e) + alt) * sin(radians(lat));
			
			Vector3d coordECEF(x, y, z);
			return coordECEF;
		}
		
		Vector3d convertECEFtoENU(Vector3d coordECEF) {
			double rad_lat_r = radians(lat_r), rad_lon_r = radians(lon_r);
			Matrix3d mat;

			mat << -sin(rad_lon_r), cos(rad_lon_r), 0,
				   -sin(rad_lat_r) * cos(rad_lon_r), -sin(rad_lat_r) * sin(rad_lon_r),
				   cos(rad_lat_r), cos(rad_lat_r) * cos(rad_lon_r),  cos(rad_lat_r) * sin(rad_lon_r),  sin(rad_lat_r);
			
			Vector3d coordENU = mat * (coordECEF - coordECEF_r);
			return coordENU;
		}
		
		void gpsConvertCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
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
			
			Vector3d heading;
			tf::Quaternion rotation;

			if (!first_message_received) { rotation.setRPY(0.0, 0.0, 0.0);  } // No rotation;
			else {
				if (coords == last_coords) { heading = last_heading; } // Has the robot moved since last message received?
				else {
					heading = computeHeading(coords, last_coords);
					last_heading = heading;
				}
				// ROS_INFO("Heading: %f, %f, %f\n", heading(0), heading(1), heading(2));
				rotation = computeRotation(heading);
			}
			tf::quaternionTFToMsg(rotation, odom_msg.pose.pose.orientation);
			
			last_coords = coords;
			first_message_received = true;
			
			pub.publish(odom_msg);
			// ROS_INFO("Pub odom => pose:[x: %f, y: %f, z: %f]; orientation:[x: %f, y: %f, z: %f, w: %f]\n",

			// 	odom_msg.pose.pose.position.x,
			// 	odom_msg.pose.pose.position.y,
			// 	odom_msg.pose.pose.position.z,

			// 	odom_msg.pose.pose.orientation.x,
			// 	odom_msg.pose.pose.orientation.y,
			// 	odom_msg.pose.pose.orientation.z,
			// 	odom_msg.pose.pose.orientation.w
			// );
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "gps_to_odom");
	Gps_To_Odom gps_to_odom;
	ros::spin();
	return 0;
}