#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

struct Pose {
    double x, y, theta;
};

class CoveragePathPlanner {
public:
    void init(ros::NodeHandle& nh);
    std::vector<Pose> generateBowPath();

private:
    ros::Timer path_timer;
    ros::Publisher coverage_path_pub;

    double squareSize_ = 10.0;          // Size of the square area
    double stepSize_ = 1.0;             // Distance between rows
    double turningRadius_ = 2.0;        // Minimum turning radius for RS curves (not used directly here)

    std::vector<Pose> _path;

    void generateSegment(std::vector<Pose> &path, const Pose &start, const Pose &end);
    void rcvWpsCallBack(const ros::TimerEvent& event);
};