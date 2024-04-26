#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Quaternion.h>

using namespace std;

class Odom_To_TF {

	private:
		ros::Subscriber sub_odom; // Sub;
        tf::TransformBroadcaster tf_pub; // TF Pub;
        string node_name; // The name of the current node, set in the launch file;
        string root_frame; // The frame_id to use in the tf, set in the launch file;
        string child_frame; // The child_frame_id to use in the tf, set in the launch file;
	
	public:

		// Constructor;
		Odom_To_TF() {
			ros::NodeHandle nh;
            node_name = ros::this_node::getName(); // Name of the current node (set in the launch file), used to retrieve the correct parameter from ROS parameter server when multiple node of this type are running;

            // Get variables from ROS parameter server, those set in the launch file;
            nh.getParam(node_name + "/root_frame", root_frame);
            nh.getParam(node_name + "/child_frame", child_frame);

            sub_odom = nh.subscribe("input_odom", 100, &Odom_To_TF::odomConvertCallback, this);
		}

        // Callback when a odometry message is received;
        void odomConvertCallback(const nav_msgs::Odometry::ConstPtr& msg) {
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)); // Set position of transform message to the coordinates received in the message;

            tf::Quaternion rotation;
            tf::quaternionMsgToTF(msg->pose.pose.orientation, rotation); // Transform the geometry_msgs quaternion we created to TF which is the type required by the transform message we want to pub;
            transform.setRotation(rotation); // Set rotation of message with the quaternion;

            tf_pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame)); // Publish transform message;

            // ROS_INFO("Node2_INFO(%s): Translation: [x: %f, y: %f, z: %f]", node_name.c_str(), transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            // rotation = transform.getRotation();
            // ROS_INFO("Node2_INFO(%s): Rotation: [x: %f, y: %f, z: %f, w: %f]\n\n", node_name.c_str(), rotation.x(), rotation.y(), rotation.z(), rotation.w());
        }
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "odom_to_tf"); // Init;
	Odom_To_TF odom_To_TF; // Call class constructor;
	ros::spin(); // Call spin;
	return 0;
}