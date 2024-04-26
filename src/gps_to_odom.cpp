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

		double epsilon = 0.05;

		
		double radians(double degrees) { return degrees * M_PI / 180.0; }
		
		double computeESquare() { return 1 - (pow(b, 2) / pow(a, 2)); }
		
		double computeN(double phi, double eSquare) { return a / sqrt(1 - eSquare * pow(sin(radians(phi)), 2)); }

		Vector3d rotateVector(Vector3d vector) {
			double tetha = radians(128.5);
			Matrix3d rotation;
			rotation << cos(tetha), -sin(tetha), 0,
						sin(tetha), cos(tetha), 0,
						0, 0, 1;
			return rotation * vector;
		}
		
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

			rotation.setRPY(0, 0, yaw);
			rotation.normalized();
			return rotation;
		}
	
	public:
		
		Gps_To_Odom() {
			sub = nh.subscribe("/fix", 100, &Gps_To_Odom::gpsConvertCallback, this);
			pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 100);
			
			nh.getParam("gps_to_odom/lat_r", lat_r);
			nh.getParam("gps_to_odom/lon_r", lon_r);
			nh.getParam("gps_to_odom/alt_r", alt_r);
			
			Vector3d refPoint(lat_r, lon_r, alt_r);
			coordECEF_r = convertGPStoECEF(refPoint);
		}
		
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
		
		Vector3d convertECEFtoENU(Vector3d coordECEF) {
			double rad_lat_r = radians(lat_r), rad_lon_r = radians(lon_r);
			Matrix3d mat;

			mat << -sin(rad_lon_r), cos(rad_lon_r), 0,
				   -sin(rad_lat_r) * cos(rad_lon_r), -sin(rad_lat_r) * sin(rad_lon_r), cos(rad_lat_r),
				   cos(rad_lat_r) * cos(rad_lon_r),  cos(rad_lat_r) * sin(rad_lon_r),  sin(rad_lat_r);
			
			Vector3d result = mat * (coordECEF - coordECEF_r);

			return rotateVector(result);
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

			if (!first_message_received) { rotation.setRPY(0.0, 0.0, 0.0); heading << 0.0, 0.0, 0.0;  } // No rotation;
			else {
				if ((coords - last_coords).norm() <= epsilon) { heading = last_heading; } // Has the robot moved ENOUGH since last message received?
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
	ros::init(argc, argv, "gps_to_odom");
	Gps_To_Odom gps_to_odom;
	ros::spin();
	return 0;
}