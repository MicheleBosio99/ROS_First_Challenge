#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Quaternion.h>

using namespace std;

class Odom_To_TF {

	private:
		ros::Subscriber sub_odom;
        tf::TransformBroadcaster tf_pub;
        std::string root_frame;
        std::string child_frame;
	
	public:
		
		Odom_To_TF() {
			ros::NodeHandle nh;
            nh.getParam("root_frame", root_frame);
            nh.getParam("child_frame", child_frame);

            sub_odom = nh.subscribe("input_odom", 10, &Odom_To_TF::odomConvertCallback, this);
		}

        void odomConvertCallback(const nav_msgs::Odometry::ConstPtr& msg) {
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

            tf::Quaternion rotation;
            tf::quaternionMsgToTF(msg->pose.pose.orientation, rotation);
            transform.setRotation(rotation);

            tf_pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame));


            ROS_INFO("Translation: [x: %f, y: %f, z: %f]", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            rotation = transform.getRotation();
            ROS_INFO("Rotation: [x: %f, y: %f, z: %f, w: %f]\n\n", rotation.x(), rotation.y(), rotation.z(), rotation.w());
        }
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "odom_to_tf");
	Odom_To_TF odom_To_TF;
	ros::spin();
	return 0;
}