#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>


class RobotWrench
{
private:
    double alpha;       // Slope angle
    double x, y;        // Coordinates of the center of gravity relative to the rear wheels
    double d, L;        // Wheelbase, vehicle length
    double a_x, a_y, a_l, a_r; // Accelerations
    double m, m_r, m_rr, m_rl, g; // Mass, gravity parameters

    ros::Subscriber imu_sub;
    gazebo::transport::SubscriberPtr sub;

    // Collision names corresponding to each wheel
    // std::string wheel_front_right_string = "willand::wheel_1::wheel_1_collision";
    // std::string wheel_rear_right_string  = "willand::wheel_2::wheel_2_collision";
    // std::string wheel_front_left_string  = "willand::wheel_3::wheel_3_collision";
    // std::string wheel_rear_left_string   = "willand::wheel_4::wheel_4_collision";
    std::string wheel_front_right_string = "willand::wheel_front_right_1::wheel_front_right_1_collision";
    std::string wheel_rear_right_string  = "willand::wheel_rear_right_1::wheel_rear_right_1_collision";
    std::string wheel_front_left_string  = "willand::wheel_front_left_1::wheel_front_left_1_collision";
    std::string wheel_rear_left_string   = "willand::wheel_rear_left_1::wheel_rear_left_1_collision";

    visualization_msgs::Marker marker;

    ros::Publisher pub_F_l_;
    ros::Publisher pub_N_l_;
    ros::Publisher pub_F_r_;
    ros::Publisher pub_N_r_;

    ros::Publisher rear_left_marker_pub;
    ros::Publisher rear_right_marker_pub;
    ros::Publisher front_left_marker_pub;
    ros::Publisher front_right_marker_pub;

    // Publish the forces and normal forces
    std_msgs::Float64 msg_F_l, msg_N_l, msg_F_r, msg_N_r;

    geometry_msgs::Point wheel_rear_left_position;
    geometry_msgs::Point wheel_rear_right_position;
    geometry_msgs::Point wheel_front_left_position;
    geometry_msgs::Point wheel_front_right_position;

public:
    double yaw, pitch, roll;     // Euler angles
    double Nf_car, Nr_car, Nrl_car, Nrr_car; // Normal forces
    double F_dl, F_fl, F_l, N_l; // driving and friction forces of left rear wheel
    double F_dr, F_fr, F_r, N_r; // driving and friction forces of right rear wheel
    double mu_sim_l, mu_sim_r;
    double mu_model_l, mu_model_r;

    geometry_msgs::Wrench wheel_rear_left_wrench;
    geometry_msgs::Wrench wheel_rear_right_wrench;
    geometry_msgs::Wrench wheel_front_left_wrench;
    geometry_msgs::Wrench wheel_front_right_wrench;

    geometry_msgs::Vector3 rl_force_wheel, rr_force_wheel;

    void computeWrench(); // Compute the forces and moments
    void wrenchCompare();
    void init(ros::NodeHandle& nh, gazebo::transport::NodePtr node); // Initialize the class
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg); // Callback to process IMU data
    void wrenchCb(ConstContactsPtr &_msg);

    geometry_msgs::Vector3 transformForceToVehicleFrame(const geometry_msgs::Vector3 &force_world);
    visualization_msgs::Marker createArrowMarker(int id, const std::string& frame_id, const geometry_msgs::Vector3& force, const geometry_msgs::Point& position);

};

