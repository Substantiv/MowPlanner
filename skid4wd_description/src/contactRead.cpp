#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <iostream>
#include <vector>

// ROS Publishers for each wheel's wrench
ros::Publisher pub_wheel_rear_left_wrench;
ros::Publisher pub_wheel_rear_right_wrench;
ros::Publisher pub_wheel_front_left_wrench;
ros::Publisher pub_wheel_front_right_wrench;

// Declare wrench variables for each wheel
static geometry_msgs::Wrench wheel_rear_left_wrench;
static geometry_msgs::Wrench wheel_rear_right_wrench;
static geometry_msgs::Wrench wheel_front_left_wrench;
static geometry_msgs::Wrench wheel_front_right_wrench;

// Callback function for contact messages
void forcesCb(ConstContactsPtr &_msg){

    // Collision names corresponding to each wheel
    std::string wheel_rear_left_string   = "skid4wd::wheel_rear_left_1::wheel_rear_left_1_collision";
    std::string wheel_rear_right_string  = "skid4wd::wheel_rear_right_1::wheel_rear_right_1_collision";
    std::string wheel_front_left_string  = "skid4wd::wheel_front_left_1::wheel_front_left_1_collision";
    std::string wheel_front_right_string = "skid4wd::wheel_front_right_1::wheel_front_right_1_collision";

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

    // Publish the wrench data for each wheel
    pub_wheel_rear_left_wrench.publish(wheel_rear_left_wrench);
    pub_wheel_rear_right_wrench.publish(wheel_rear_right_wrench);
    pub_wheel_front_left_wrench.publish(wheel_front_left_wrench);
    pub_wheel_front_right_wrench.publish(wheel_front_right_wrench);
}

int main(int _argc, char **_argv){
    // Initialize Gazebo client and ROS
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "contact_read_node");

    // Set up Gazebo transport node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", forcesCb);

    // Initialize ROS node handle and publishers
    ros::NodeHandle n;
    pub_wheel_rear_left_wrench   = n.advertise<geometry_msgs::Wrench>("/ninebot/wheel_rear_left_wrench", 1000);
    pub_wheel_rear_right_wrench  = n.advertise<geometry_msgs::Wrench>("/ninebot/wheel_rear_right_wrench", 1000);
    pub_wheel_front_left_wrench  = n.advertise<geometry_msgs::Wrench>("/ninebot/wheel_front_left_wrench", 1000);
    pub_wheel_front_right_wrench = n.advertise<geometry_msgs::Wrench>("/ninebot/wheel_front_right_wrench", 1000);

    // Main loop: continuously check and publish forces and torques
    while (ros::ok())
    {
        gazebo::common::Time::MSleep(20);  // Sleep for 20 ms to reduce CPU usage
        ros::spinOnce();  // Handle incoming ROS messages
    }

    // Shutdown Gazebo client
    gazebo::client::shutdown();
}
