#include "robot_wrench.h"

void RobotWrench::init(ros::NodeHandle& nh, gazebo::transport::NodePtr node)
{
    m = 69;           // Vehicle mass
    g = 9.81;         // Gravitational acceleration
    alpha = 0.2;      // Convert angle from degrees to radians
    L = 0.5;          // Vehicle length
    x = 0.5 * L;      // Distance of the center of gravity from the rear axle
    y = 0.1;          // Distance of the center of gravity from the lateral centerline
    d = 0.3;          // Distance between the left and right wheels

    imu_sub = nh.subscribe<sensor_msgs::Imu>("/willand/imu/data", 10, &RobotWrench::imuCallback, this);
    sub = node->Subscribe("/gazebo/default/physics/contacts", &RobotWrench::wrenchCb, this);
}

void RobotWrench::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;

    // Compute roll angle
    roll = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));

    // Compute pitch angle
    pitch = asin(2.0 * (qw * qy - qz * qx));

    // Compute yaw angle
    yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    // TODO:角度还需要进一步处理 
    yaw = yaw - 1.57;

    a_x = msg->linear_acceleration.x; // Linear acceleration in the x-direction
    a_y = msg->linear_acceleration.y; // Linear acceleration in the y-direction

    // Compute angular acceleration
    static ros::Time last_time;
    static double last_angular_velocity_z = 0.0;
    double angular_acceleration = 0.0;
    ros::Time current_time = ros::Time::now();
    double current_angular_velocity_z = msg->angular_velocity.z; // Angular velocity around z-axis

    if (!last_time.isZero())
    {
        double delta_omega = current_angular_velocity_z - last_angular_velocity_z;
        double delta_t = (current_time - last_time).toSec();

        if (delta_t > 0)
        {
            angular_acceleration = delta_omega / delta_t; // Compute angular acceleration
        }
    }

    last_angular_velocity_z = current_angular_velocity_z;
    last_time = current_time;

    // Compute left and right wheel accelerations
    a_l = a_x - (angular_acceleration * d / 2.0); // Left wheel acceleration
    a_r = a_x + (angular_acceleration * d / 2.0); // Right wheel acceleration
}

// Callback function for contact messages
void RobotWrench::wrenchCb(ConstContactsPtr &_msg){

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

void RobotWrench::wrenchCompare(){

    computeWrench();

    rl_force_wheel = transformForceToVehicleFrame(wheel_rear_left_wrench.force);
    rr_force_wheel = transformForceToVehicleFrame(wheel_rear_right_wrench.force);

    mu_sim_l = sqrt(std::pow(rl_force_wheel.x,2) + std::pow(rl_force_wheel.y,2)) / rl_force_wheel.z;
    mu_sim_r = sqrt(std::pow(rr_force_wheel.x,2) + std::pow(rr_force_wheel.y,2)) / rr_force_wheel.z;
    mu_model_l = F_l / N_l;
    mu_model_r = F_r / N_r;
}

void RobotWrench::computeWrench()
{
    /* Compute normal forces */
    Nf_car = (m * g * x * cos(pitch) - m * g * y * sin(pitch) - m * y * a_y) / L;
    Nr_car = m * g * cos(pitch) - Nf_car;
    m_r = Nr_car / (Nf_car + Nr_car);

    Nrl_car = 0.5 * m_r * g * cos(roll) - y / d * m_r * g * sin(roll) + m_r * a_y * y / d;
    Nrr_car = Nr_car - Nrl_car;
    m_rl = Nrl_car / (Nf_car + Nr_car);
    m_rr = Nrr_car / (Nf_car + Nr_car);

    /* Compute driving and friction forces */
    F_dl = m_rl * g * sin(pitch) + m_rl * a_l;
    F_fl = m_rl * g * sin(alpha) + m_rl * a_l * cos(yaw);
    F_l = sqrt(F_dl * F_dl + F_fl * F_fl + 2 * F_dl * F_fl * cos(yaw));
    N_l = m_rl * g * cos(alpha);

    F_dr = m_rr * g * sin(pitch) + m_rr * a_r;
    F_fr = m_rr * g * sin(alpha) + m_rr * a_r * cos(yaw);
    F_r = sqrt(F_dr * F_dr + F_fr * F_fr + 2 * F_dr * F_fr * cos(yaw));
    N_r = m_rr * g * cos(alpha);
}

geometry_msgs::Vector3 RobotWrench::transformForceToVehicleFrame(
    const geometry_msgs::Vector3 &force_world) {

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
