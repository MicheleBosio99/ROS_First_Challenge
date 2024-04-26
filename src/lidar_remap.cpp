#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>
#include <pcl_ros/transforms.h>

using namespace std;

string frame_id;
ros::Publisher pub;

void callback(first_project::parametersConfig &config, uint32_t level) {
    if (config.odom == "wheel_odom" || config.odom == "gps_odom") {
        frame_id = config.odom;
    }
}

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    sensor_msgs::PointCloud2 modified_msg = *msg;
    modified_msg.header.frame_id = frame_id;
    modified_msg.header.stamp = ros::Time::now();
    pub.publish(modified_msg);
    ROS_INFO("Node3: Published PointCloud2 msg with header: %s", frame_id.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<first_project::parametersConfig> server;
    dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Subscriber sub = nh.subscribe("/os_cloud_node/points", 100, lidarCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);

    ros::spin();
    return 0;
}