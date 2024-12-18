#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "robot_wrench.h"

#include <mu_values.pb.h> 

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>


// Declare wrench variables for each wheel
static geometry_msgs::Wrench wheel_rear_left_wrench;
static geometry_msgs::Wrench wheel_rear_right_wrench;
static geometry_msgs::Wrench wheel_front_left_wrench;
static geometry_msgs::Wrench wheel_front_right_wrench;

RobotWrench willand_robot;

geometry_msgs::Vector3 transformForceToVehicleFrame(
    const geometry_msgs::Vector3 &force_world,
    double yaw, double pitch, double roll) {

    // Create rotation matrices
    Eigen::Matrix3d Rz, Ry, Rx, R;

    // Yaw rotation (around Z axis)
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw),  cos(yaw), 0,
          0,         0,        1;

    // Pitch rotation (around Y axis)
    Ry << cos(pitch), 0, sin(pitch),
          0,          1, 0,
         -sin(pitch), 0, cos(pitch);

    // Roll rotation (around X axis)
    Rx << 1, 0,         0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);

    // Combined rotation matrix
    R = Rz * Ry * Rx;

    // Force in world frame
    Eigen::Vector3d force_world_vec(force_world.x, force_world.y, force_world.z);

    // Transform to vehicle frame
    Eigen::Vector3d force_vehicle_vec = R * force_world_vec;

    // Convert back to geometry_msgs::Vector3
    geometry_msgs::Vector3 force_vehicle;
    force_vehicle.x = force_vehicle_vec(0);
    force_vehicle.y = force_vehicle_vec(1);
    force_vehicle.z = force_vehicle_vec(2);

    return force_vehicle;
}

void wrenchCompare(){
    willand_robot.computeWrench();

    double mu_sim_l, mu_sim_r;
    double mu_model_l, mu_model_r;
    geometry_msgs::Vector3 rl_force_wheel, rr_force_wheel;

    rl_force_wheel = transformForceToVehicleFrame(
                    wheel_rear_left_wrench.force, 
                    willand_robot.yaw, willand_robot.pitch, willand_robot.roll);
    rl_force_wheel = transformForceToVehicleFrame(
                    wheel_rear_right_wrench.force, 
                    willand_robot.yaw, willand_robot.pitch, willand_robot.roll);

    mu_sim_l = sqrt(std::pow(rl_force_wheel.x,2) + std::pow(rl_force_wheel.y,2)) / rl_force_wheel.z;
    mu_sim_r = sqrt(std::pow(rr_force_wheel.x,2) + std::pow(rr_force_wheel.y,2)) / rr_force_wheel.z;
    mu_model_l = willand_robot.F_l / willand_robot.N_l;
    mu_model_r = willand_robot.F_r / willand_robot.N_r;

    willand_description::MuValues mu_values;
    mu_values.set_mu_sim_l(mu_sim_l);
    mu_values.set_mu_sim_r(mu_sim_r);
    mu_values.set_mu_model_l(mu_model_l);
    mu_values.set_mu_model_r(mu_model_r);

    std::ofstream output_file("/home/jiance/Development/willand_ws/src/willand_description/lib/mu_values.bin", std::ios::out | std::ios::binary);

    if (!mu_values.SerializeToOstream(&output_file)) {
        std::cerr << "Failed to write mu values to file." << std::endl;
    }
    ROS_INFO("Here");

    output_file.close();
}

// Callback function for contact messages
void wrenchCb(ConstContactsPtr &_msg){

    // Collision names corresponding to each wheel
    std::string wheel_rear_left_string   = "willand::wheel_rear_left_1::wheel_rear_left_1_collision";
    std::string wheel_rear_right_string  = "willand::wheel_rear_right_1::wheel_rear_right_1_collision";
    std::string wheel_front_left_string  = "willand::wheel_front_left_1::wheel_front_left_1_collision";
    std::string wheel_front_right_string = "willand::wheel_front_right_1::wheel_front_right_1_collision";

    // Iterate over all contacts in the message
    for (int i = 0; i < _msg->contact_size(); ++i) {

        // Check if the contact is related to the rear left wheel
        if(_msg->contact(i).collision1() == wheel_rear_left_string){
            wheel_rear_left_wrench.force.x = _msg->contact(i).wrench(0).body_1_wrench().force().x();
            wheel_rear_left_wrench.force.y = _msg->contact(i).wrench(0).body_1_wrench().force().y();
            wheel_rear_left_wrench.force.z = _msg->contact(i).wrench(0).body_1_wrench().force().z();
            wheel_rear_left_wrench.torque.x = _msg->contact(i).wrench(0).body_1_wrench().torque().x();
            wheel_rear_left_wrench.torque.y = _msg->contact(i).wrench(0).body_1_wrench().torque().y();
            wheel_rear_left_wrench.torque.z = _msg->contact(i).wrench(0).body_1_wrench().torque().z();
        }

        // Check if the contact is related to the rear right wheel
        if(_msg->contact(i).collision1() == wheel_rear_right_string){
           
            wheel_rear_right_wrench.force.x = _msg->contact(i).wrench(0).body_1_wrench().force().x();
            wheel_rear_right_wrench.force.y = _msg->contact(i).wrench(0).body_1_wrench().force().y();
            wheel_rear_right_wrench.force.z = _msg->contact(i).wrench(0).body_1_wrench().force().z();
            wheel_rear_right_wrench.torque.x = _msg->contact(i).wrench(0).body_1_wrench().torque().x();
            wheel_rear_right_wrench.torque.y = _msg->contact(i).wrench(0).body_1_wrench().torque().y();
            wheel_rear_right_wrench.torque.z = _msg->contact(i).wrench(0).body_1_wrench().torque().z();
        }

        // Check if the contact is related to the front left wheel
        if(_msg->contact(i).collision1() == wheel_front_left_string){
            wheel_front_left_wrench.force.x = _msg->contact(i).wrench(0).body_1_wrench().force().x();
            wheel_front_left_wrench.force.y = _msg->contact(i).wrench(0).body_1_wrench().force().y();
            wheel_front_left_wrench.force.z = _msg->contact(i).wrench(0).body_1_wrench().force().z();
            wheel_front_left_wrench.torque.x = _msg->contact(i).wrench(0).body_1_wrench().torque().x();
            wheel_front_left_wrench.torque.y = _msg->contact(i).wrench(0).body_1_wrench().torque().y();
            wheel_front_left_wrench.torque.z = _msg->contact(i).wrench(0).body_1_wrench().torque().z();
        }

        // Check if the contact is related to the front right wheel
        if(_msg->contact(i).collision1() == wheel_front_right_string){
            wheel_front_right_wrench.force.x = _msg->contact(i).wrench(0).body_1_wrench().force().x();
            wheel_front_right_wrench.force.y = _msg->contact(i).wrench(0).body_1_wrench().force().y();
            wheel_front_right_wrench.force.z = _msg->contact(i).wrench(0).body_1_wrench().force().z();
            wheel_front_right_wrench.torque.x = _msg->contact(i).wrench(0).body_1_wrench().torque().x();
            wheel_front_right_wrench.torque.y = _msg->contact(i).wrench(0).body_1_wrench().torque().y();
            wheel_front_right_wrench.torque.z = _msg->contact(i).wrench(0).body_1_wrench().torque().z();
        }
    }

    wrenchCompare();
}

int main(int _argc, char **_argv){
    // Initialize Gazebo client and ROS
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "robot_wrench_node");

    // Set up Gazebo transport node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", wrenchCb);

    // Initialize ROS node handle
    ros::NodeHandle n;
    willand_robot.init(n);


    // Main loop: continuously check and publish forces and torques
    while (ros::ok())
    {
        gazebo::common::Time::MSleep(20);  // Sleep for 20 ms to reduce CPU usage
        ros::spinOnce();  // Handle incoming ROS messages
    }

    // Shutdown Gazebo client
    gazebo::client::shutdown();
}
