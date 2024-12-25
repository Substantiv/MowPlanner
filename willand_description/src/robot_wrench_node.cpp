#include "robot_wrench.h"

int main(int _argc, char **_argv){
    // Initialize Gazebo client and ROS
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "robot_wrench_node");

    // Set up Gazebo transport node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Initialize ROS node handle
    ros::NodeHandle n;

    RobotWrench willand_robot;
    willand_robot.init(n, node);

    ros::Rate loop_rate(50);

    // Main loop
    while (ros::ok())
    {
        gazebo::common::Time::MSleep(20);
        ros::spinOnce();
        loop_rate.sleep();
    }

    gazebo::client::shutdown();
}
