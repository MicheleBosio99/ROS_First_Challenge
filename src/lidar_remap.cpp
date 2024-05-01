<<<<<<< Updated upstream
=======
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>
#include <pcl_ros/transforms.h>

using namespace std;

string frame_id;
ros::Publisher pub;

// Callback from the reconfigure server whenever a parameter has been changed;
void callback(first_project::parametersConfig &config, uint32_t level) {
    // Just set the frame_id we are using to the value received (the if statement is just to prevent errors if the value received is not correct for whatever reason);
    if (config.odom == "wheel_odom" || config.odom == "gps_odom") {
        frame_id = config.odom;
    }
}

// Callback from the subscriber to the PointCloud2 messages
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    sensor_msgs::PointCloud2 modified_msg = *msg; // Modified message variable copies the message received as it is, since the msg received in the callback is only for reads and not writes;

    modified_msg.header.frame_id = frame_id; // Change frame_id to the current one it's inside the reconfigure server;
    modified_msg.header.stamp = ros::Time::now(); // Give the message a new timestamp in the header (had a problem for which messages where not considered in rviz because they arrived too late for it to visualize them; solved with this line);

    pub.publish(modified_msg); // Publish message;
    ROS_INFO("NODE3: Published PointCloud2 msg with header: %s", frame_id.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap"); // Init;
    ros::NodeHandle nh; // Declare node handler variable;

    // All the reconfigure parameters are set in the parameters.cfg file;
    dynamic_reconfigure::Server<first_project::parametersConfig> server; // Declare reconfigure server;
    dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f; // Declare callback type for the reconfigure;
    f = boost::bind(&callback, _1, _2); // Set the callback function of the reconfigure type variable;
    server.setCallback(f); // Set the callback type in the server;

    ros::Subscriber sub = nh.subscribe("/os_cloud_node/points", 100, lidarCallback); // Subscribe to PointCloud2 messages;
    pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1); // Publish the PointCloud modified messages on a new topic;

    ros::spin(); // Spin;
    return 0;
}
>>>>>>> Stashed changes
