#pragma once

#include <thread>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include "utils/lbfgs.hpp"
#include "utils/se2traj.hpp"

class ALMTrajOpt
{
private:
    ros::Subscriber coverage_path_sub;
    ros::Publisher se2_traj_pub;    

public:
    // params

    /// problem
    double rho_T;
    double rho_ter;
    double max_vel;
    double max_acc_lon;
    double max_acc_lat;
    double max_kap;
    double min_cxi;
    double max_sig;

    /// ALM
    bool   use_scaling;
    double rho;
    double beta;
    double gamma;
    double epsilon_con;
    double max_iter;

    /// L-BFGS
    double g_epsilon;
    double min_step;
    double inner_max_iter;
    double delta;
    int    mem_size;
    int    past;
    int    int_K;

    // data
    int             piece_xy;
    int             piece_yaw;
    int             dim_T;
    double          equal_num;
    double          non_equal_num;
    double          scale_fx;   // Scaling factor for the objective function
    Eigen::VectorXd scale_cx;   // Scaling factors for the constraints
    Eigen::VectorXd hx;         // Stores the values of equality constraints
    Eigen::VectorXd gx;         // Stores the values of inequality constraints
    Eigen::VectorXd lambda;     // Stores the Lagrange multipliers for equality constraints
    Eigen::VectorXd mu;         // Stores the Lagrange multipliers for inequality constraints
    Eigen::MatrixXd init_xy;
    Eigen::MatrixXd end_xy;
    Eigen::MatrixXd init_yaw;
    Eigen::MatrixXd end_yaw;
    MINCO_SE2       minco_se2;

    void init(ros::NodeHandle& nh);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    int optimizeTraj(const Eigen::MatrixXd &initStateXY, const Eigen::MatrixXd &endStateXY, const Eigen::MatrixXd &innerPtsXY, \
                    const Eigen::VectorXd &initYaw,const Eigen::VectorXd &endYaw, const Eigen::VectorXd &innerPtsYaw, \
                    const double & totalTime);
    void visSE2Traj(const SE2Trajectory& traj);

    inline double logC2(const double& T);
    inline double expC2(const double& tau);
    inline double getTtoTauGrad(const double& tau);
    inline void updateDualVars();
    inline bool judgeConvergence();
    inline void calTfromTau(const double& tau, Eigen::VectorXd& T);

};

// tau = ln(T)
inline double ALMTrajOpt::logC2(const double& T)
{
    return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}

// T = e^tau
inline double ALMTrajOpt::expC2(const double& tau)
{
    return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0) : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
}

inline void ALMTrajOpt::updateDualVars()
{
    lambda += rho * hx;
    for(int i = 0; i < non_equal_num; i++)
        mu(i) = std::max(mu(i)+rho*gx(i), 0.0);
    rho = std::min((1 + gamma) * rho, beta);
}

inline bool ALMTrajOpt::judgeConvergence()
{
    std::cout << "reshx: "<<hx.lpNorm<Eigen::Infinity>() << " resgx: "<<gx.cwiseMax(-mu/rho).lpNorm<Eigen::Infinity>() << std::endl;
    
    if (std::max(hx.lpNorm<Eigen::Infinity>(), gx.cwiseMax(-mu/rho).lpNorm<Eigen::Infinity>()) < epsilon_con)
        return true;

    return false;
}

// know tau then get T (uniform)
inline void ALMTrajOpt::calTfromTau(const double& tau, Eigen::VectorXd& T)
{
    T.setConstant(expC2(tau) / T.size());
    return;
}

inline double ALMTrajOpt::getTtoTauGrad(const double& tau)
{
    if (tau > 0)
        return tau + 1.0;
    else 
    {
        double denSqrt = (0.5 * tau - 1.0) * tau + 1.0;
        return (1.0 - tau) / (denSqrt * denSqrt);
    } 
}

static double innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad);

static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                        const double fx, const double step, int k, int ls);