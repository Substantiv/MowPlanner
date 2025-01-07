#include "alm_traj_opt.h"


void ALMTrajOpt::init(ros::NodeHandle& nh){
    nh.getParam("alm_traj_opt/rho_T", rho_T);
    nh.getParam("alm_traj_opt/rho_ter", rho_ter);
    nh.getParam("alm_traj_opt/max_vel", max_vel);
    nh.getParam("alm_traj_opt/max_acc_lon", max_acc_lon);
    nh.getParam("alm_traj_opt/max_acc_lat", max_acc_lat);
    nh.getParam("alm_traj_opt/max_kap", max_kap);
    nh.getParam("alm_traj_opt/min_cxi", min_cxi);
    nh.getParam("alm_traj_opt/max_sig", max_sig);
    nh.getParam("alm_traj_opt/use_scaling", use_scaling);
    nh.getParam("alm_traj_opt/rho", rho);
    nh.getParam("alm_traj_opt/beta", beta);
    nh.getParam("alm_traj_opt/gamma", gamma);
    nh.getParam("alm_traj_opt/epsilon_con", epsilon_con);
    nh.getParam("alm_traj_opt/max_iter", max_iter);
    nh.getParam("alm_traj_opt/g_epsilon", g_epsilon);
    nh.getParam("alm_traj_opt/min_step", min_step);
    nh.getParam("alm_traj_opt/inner_max_iter", inner_max_iter);
    nh.getParam("alm_traj_opt/delta", delta);
    nh.getParam("alm_traj_opt/mem_size", mem_size);
    nh.getParam("alm_traj_opt/past", past);
    nh.getParam("alm_traj_opt/int_K", int_K);

    coverage_path_sub = nh.subscribe<nav_msgs::Path>("/coverage_path", 10, &ALMTrajOpt::pathCallback, this);
}


void ALMTrajOpt::pathCallback(const nav_msgs::Path::ConstPtr& msg){

    // process the inital path
    std::vector<Eigen::Vector3d> init_path;
    for(size_t i=0; i < msg->poses.size(); i++){
        Eigen::Vector3d pose_temp;
        double roll, pitch, yaw;
        
        pose_temp[0] = msg->poses[i].pose.position.x;
        pose_temp[1] = msg->poses[i].pose.position.y;
        
        tf::Quaternion q(
            msg->poses[i].pose.orientation.x,
            msg->poses[i].pose.orientation.y,
            msg->poses[i].pose.orientation.z,
            msg->poses[i].pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        pose_temp[2] = yaw;

        init_path.push_back(pose_temp);
    }
    
    // smooth yaw
    double dyaw;
    for (size_t i=0; i<init_path.size()-1; i++)
    {
        dyaw = init_path[i+1].z() - init_path[i].z();
        while (dyaw >= M_PI / 2)
        {
            init_path[i+1].z() -= M_PI * 2;
            dyaw = init_path[i+1].z() - init_path[i].z();
        }
        while (dyaw <= -M_PI / 2)
        {
            init_path[i+1].z() += M_PI * 2;
            dyaw = init_path[i+1].z() - init_path[i].z();
        }
    }

    // init solution
    Eigen::Matrix<double, 2, 3> init_xy, end_xy;
    Eigen::Vector3d init_yaw, end_yaw;
    Eigen::MatrixXd inner_xy;
    Eigen::VectorXd inner_yaw;
    double total_time;

    init_xy << init_path[0].x(), 0.0, 0.0, \
                init_path[0].y(), 0.0, 0.0;
    end_xy << init_path.back().x(), 0.0, 0.0, \
                init_path.back().y(), 0.0, 0.0;
    init_yaw << init_path[0].z(), 0.0, 0.0;
    end_yaw << init_path.back().z(), 0.0, 0.0;

    init_xy.col(1) << 0.05 * cos(init_yaw(0)), 0.05 * sin(init_yaw(0));
    end_xy.col(1) << 0.05 * cos(end_yaw(0)), 0.05 * sin(end_yaw(0));

    // linear interpolation for position and yaw    
    double temp_len_yaw = 0.0;
    double temp_len_pos = 0.0;
    double total_len = 0.0;
    double piece_len = 0.3;
    double piece_len_yaw = piece_len / 2.0;
    std::vector<Eigen::Vector2d> inner_xy_node;
    std::vector<double> inner_yaw_node;
    for (int k=0; k<init_path.size()-1; k++)
    {
        double temp_seg = (init_path[k+1] - init_path[k]).head(2).norm();
        temp_len_yaw += temp_seg;
        temp_len_pos += temp_seg;
        total_len += temp_seg;
        if (temp_len_yaw > piece_len_yaw)
        {
            double temp_yaw = init_path[k].z() + (1.0 - (temp_len_yaw-piece_len_yaw) / temp_seg) * (init_path[k+1] - init_path[k]).z();
            inner_yaw_node.push_back(temp_yaw);
            temp_len_yaw -= piece_len_yaw;
        }
        if (temp_len_pos > piece_len)
        {
            Eigen::Vector3d temp_node = init_path[k] + (1.0 - (temp_len_pos-piece_len) / temp_seg) * (init_path[k+1] - init_path[k]);
            inner_xy_node.push_back(temp_node.head(2));
            inner_yaw_node.push_back(temp_node.z());
            temp_len_pos -= piece_len;
        }
    }
    total_time = total_len / max_vel * 1.2;
    inner_xy.resize(2, inner_xy_node.size());
    inner_yaw.resize(inner_yaw_node.size());
    for (int i=0; i<inner_xy_node.size(); i++)
    {
        inner_xy.col(i) = inner_xy_node[i];
    }
    for (int i=0; i<inner_yaw_node.size(); i++)
    {
        inner_yaw(i) = inner_yaw_node[i];
    }

    optimizeTraj(init_xy, end_xy, inner_xy, init_yaw, end_yaw, inner_yaw, total_time);
}


int ALMTrajOpt::optimizeTraj(const Eigen::MatrixXd &initStateXY, const Eigen::MatrixXd &endStateXY, const Eigen::MatrixXd &innerPtsXY, \
                            const Eigen::VectorXd &initYaw, const Eigen::VectorXd &endYaw, const Eigen::VectorXd &innerPtsYaw, \
                            const double & totalTime){
    int ret_code;

    piece_xy = innerPtsXY.cols() + 1;
    piece_yaw = innerPtsYaw.size() + 1;
    init_xy = initStateXY;
    end_xy = endStateXY;
    init_yaw = initYaw.transpose();
    end_yaw = endYaw.transpose();

    int variable_num = 2*(piece_xy-1) + (piece_yaw-1) + 1;
    // non-holonomic
    equal_num = piece_xy * (int_K + 1);
    // longitude velocity, longitude acceleration, latitude acceleration, curvature, attitude
    non_equal_num = piece_xy * (int_K + 1) * 5;

    hx.resize(equal_num);
    hx.setZero();
    lambda.resize(equal_num);
    lambda.setZero();
    gx.resize(non_equal_num);
    gx.setZero();
    mu.resize(non_equal_num);
    mu.setZero();
    scale_fx = 1.0;
    scale_cx.resize(equal_num+non_equal_num);
    scale_cx.setConstant(1.0);

    // init solution
    Eigen::VectorXd x;
    x.resize(variable_num);

    dim_T = 1;
    double& tau = x(0);
    Eigen::Map<Eigen::MatrixXd> Pxy(x.data()+dim_T, 2, piece_xy-1);
    Eigen::Map<Eigen::MatrixXd> Pyaw(x.data()+dim_T+2*(piece_xy-1), 1, piece_yaw-1);

    tau = logC2(totalTime);
    Pxy = innerPtsXY;
    Pyaw = innerPtsYaw.transpose();

    // lbfgs params
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = mem_size;
    lbfgs_params.past = past;
    lbfgs_params.g_epsilon = g_epsilon;
    lbfgs_params.min_step = min_step;
    lbfgs_params.delta = delta;
    lbfgs_params.max_iterations = inner_max_iter;
    double inner_cost;

    // begin PHR Augmented Lagrangian Method
    ros::Time start_time = ros::Time::now();
    int iter = 0;

    while (true)
    {
        int result = lbfgs::lbfgs_optimize(x, inner_cost, &innerCallback, nullptr, &earlyExit, this, lbfgs_params);

        if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_CANCELED ||
            result == lbfgs::LBFGS_STOP || result == lbfgs::LBFGSERR_MAXIMUMITERATION)
        {
            ROS_INFO_STREAM("[Inner] optimization success! cost: " << inner_cost );
        }
        else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
        {
            ROS_WARN("[Inner] The line-search routine reaches the maximum number of evaluations.");
        }
        else
        {
            ret_code = 1;
            ROS_ERROR("[Inner] Solver error. Return = %d, %s.", result, lbfgs::lbfgs_strerror(result));
            break;
        }

        updateDualVars();

        if(judgeConvergence())
        {
            ROS_WARN_STREAM("[ALM] Convergence! iters: "<<iter);
            break;
        }

        if(++iter > max_iter)
        {
            ret_code = 2;
            ROS_WARN("[ALM] Reach max iteration");
            break;
        }
    }
    

    return ret_code;
}

static double innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad)
{

    double jerk_cost = 0.0;

    return jerk_cost;
}

static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                        const double fx, const double step, int k, int ls)
{

    return k > 1e3;
}
