#include <iostream>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

struct Pose {
    double x, y, theta;
};

class CoveragePathPlanner {
public:
    CoveragePathPlanner(double squareSize, double stepSize, double turningRadius)
        : squareSize_(squareSize), stepSize_(stepSize), turningRadius_(turningRadius) {}

    std::vector<Pose> generateBowPath() {
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

private:
    double squareSize_;
    double stepSize_;
    double turningRadius_;

    void generateSegment(std::vector<Pose> &path, const Pose &start, const Pose &end) {
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
};