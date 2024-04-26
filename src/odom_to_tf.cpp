#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Quaternion.h>

using namespace std;

class Odom_To_TF {

	private:
		ros::Subscriber sub_odom;
        tf::TransformBroadcaster tf_pub;
        string node_name;
        string root_frame;
        string child_frame;
	
	public:
		
		Odom_To_TF() {
			ros::NodeHandle nh;
            node_name = ros::this_node::getName();

            nh.getParam(node_name + "/root_frame", root_frame);
            nh.getParam(node_name + "/child_frame", child_frame);

            sub_odom = nh.subscribe("input_odom", 100, &Odom_To_TF::odomConvertCallback, this);
		}

        void odomConvertCallback(const nav_msgs::Odometry::ConstPtr& msg) {
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

            tf::Quaternion rotation;
            tf::quaternionMsgToTF(msg->pose.pose.orientation, rotation);
            transform.setRotation(rotation);

            tf_pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame));


            // ROS_INFO("Node2_INFO(%s): Translation: [x: %f, y: %f, z: %f]", node_name.c_str(), transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            // rotation = transform.getRotation();
            // ROS_INFO("Node2_INFO(%s): Rotation: [x: %f, y: %f, z: %f, w: %f]\n\n", node_name.c_str(), rotation.x(), rotation.y(), rotation.z(), rotation.w());
        }
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "odom_to_tf");
	Odom_To_TF odom_To_TF;
	ros::spin();
	return 0;
}