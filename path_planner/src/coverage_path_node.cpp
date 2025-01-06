#include "path_planner/coverage_path_planner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "coverage_path_publisher");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/coverage_path", 10);
    ros::Rate rate(10);

    double squareSize = 10.0;  // Size of the square area
    double stepSize = 1.0;     // Distance between rows
    double turningRadius = 2.0; // Minimum turning radius for RS curves (not used directly here)

    CoveragePathPlanner planner(squareSize, stepSize, turningRadius);
    std::vector<Pose> path = planner.generateBowPath();

    nav_msgs::Path ros_path;
    ros_path.header.frame_id = "world";
    ros_path.header.stamp = ros::Time::now();

    for (const auto &pose : path) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "world";
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = pose.x;
        pose_stamped.pose.position.y = pose.y;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);

        ros_path.poses.push_back(pose_stamped);
    }

    while (ros::ok()) {
        ros_path.header.stamp = ros::Time::now();
        path_pub.publish(ros_path);
        rate.sleep();
    }

    return 0;
}