#include "coverage_path_planner.h"

void CoveragePathPlanner::init(ros::NodeHandle& nh){
    coverage_path_pub = nh.advertise<nav_msgs::Path>("/coverage_path", 10);
    path_timer = nh.createTimer(ros::Duration(1.0), &CoveragePathPlanner::rcvWpsCallBack, this);
}

std::vector<Pose> CoveragePathPlanner::generateBowPath() {
    std::vector<Pose> path;

    bool direction = true; // True for left-to-right, false for right-to-left
    for (double y = 0; y <= squareSize_; y += stepSize_) {
        if (direction) {
            generateSegment(path, {0, y, 0}, {squareSize_, y, 0});
        } else {
            generateSegment(path, {squareSize_, y, 0}, {0, y, 0});
        }
        direction = !direction;
    }

    return path;
}

void CoveragePathPlanner::generateSegment(std::vector<Pose> &path, const Pose &start, const Pose &end){
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double steps = std::ceil(distance / stepSize_);

    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        double x = start.x + t * dx;
        double y = start.y + t * dy;
        double theta = std::atan2(dy, dx);
        path.push_back({x, y, theta});
    }
}

void CoveragePathPlanner::rcvWpsCallBack(const ros::TimerEvent& event){
    _path = generateBowPath();

    nav_msgs::Path ros_path;
    ros_path.header.frame_id = "world";
    ros_path.header.stamp = ros::Time::now();

    for (const auto &pose : _path) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "world";
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = pose.x;
        pose_stamped.pose.position.y = pose.y;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);

        ros_path.poses.push_back(pose_stamped);
    }

    coverage_path_pub.publish(ros_path);
}