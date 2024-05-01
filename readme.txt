Explanation on thoughts when coding:

Node 1:
- We used 100 as queue size to be sure all messages were received;
- We used an angle of 128.5 degrees for the rotation since we tried to compute it using the /fix and /odom topics messages and we found out it had high variance due to noise so we used the mean value;

Node 2:
- We used 100 as queue size to be sure all messages were received;
- We used this function: "tf::quaternionMsgToTF(msg->pose.pose.orientation, rotation);" to convert the quaternion from the geometry_msgs type to the TF type;

Node 3:
- We used 100 as queue size to be sure all messages were received;
- We used a enum parameter in the config file to have the 2 values needed already set and interchangeable;

Launch file:
- In the line launching RVIZ we used a modified configuration file (first_project_rviz_cfg.rviz) to already have set inside rviz the TF and PointCloud2 visualization each time it gets executed;