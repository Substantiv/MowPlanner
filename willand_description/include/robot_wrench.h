#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

class RobotWrench
{
private:
    double alpha;       // Slope angle
    double x, y;        // Coordinates of the center of gravity relative to the rear wheels
    double d, L;        // Wheelbase, vehicle length
    double a_x, a_y, a_l, a_r; // Accelerations
    double m, m_r, m_rr, m_rl, g; // Mass, gravity parameters

    ros::Subscriber imu_sub;

public:
    double yaw, pitch, roll;     // Euler angles
    double Nf_car, Nr_car, Nrl_car, Nrr_car; // Normal forces
    double F_dl, F_fl, F_l, N_l; // driving and friction forces of left rear wheel
    double F_dr, F_fr, F_r, N_r; // driving and friction forces of right rear wheel

    void computeWrench(); // Compute the forces and moments
    void init(ros::NodeHandle& nh); // Initialize the class
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg); // Callback to process IMU data
};

void RobotWrench::init(ros::NodeHandle& nh)
{
    m = 25;          // Vehicle mass
    g = 9.81;         // Gravitational acceleration
    alpha = 10 * M_PI / 180.0; // Convert angle from degrees to radians
    L = 0.8;          // Vehicle length
    x = 0.3 * L;      // Distance of the center of gravity from the rear axle
    y = 0.2;          // Distance of the center of gravity from the lateral centerline
    d = 0.6;          // Distance between the left and right wheels

    imu_sub = nh.subscribe<sensor_msgs::Imu>("/willand/imu/data", 10, &RobotWrench::imuCallback, this);
}

void RobotWrench::computeWrench()
{
    /* Compute normal forces */
    Nf_car = (m * g * x * cos(alpha) - m * g * y * sin(alpha) - m * y * a_y) / L;
    Nr_car = m * g * cos(alpha) - Nf_car;
    m_r = Nr_car / (Nf_car + Nr_car);

    Nrl_car = 0.5 * m_r * g * cos(roll) - y / d * m_r * g * sin(roll) + m_r * a_y * y;
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
