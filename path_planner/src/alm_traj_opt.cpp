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
    se2_traj_pub = nh.advertise<nav_msgs::Path>("/se2_traj", 10);
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

    SE2Trajectory back_end_traj = minco_se2.getTraj();
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
    minco_se2.reset(piece_xy, piece_yaw);

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
    lbfgs_params.mem_size = mem_size;                // 历史变量数，用于近似 Hessian 矩阵
    lbfgs_params.past = past;                        // 用于检查目标函数的收敛历史长度
    lbfgs_params.g_epsilon = g_epsilon;              // 梯度的停止准则，当梯度的无穷范数低于此值时停止迭代。
    lbfgs_params.min_step = min_step;                // 步长的最小值限制
    lbfgs_params.delta = delta;                      // 相邻目标值的变化小于此值时认为收敛
    lbfgs_params.max_iterations = inner_max_iter;    // 最大迭代次数
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


void ALMTrajOpt::visSE2Traj(const SE2Trajectory& traj)
{
    nav_msgs::Path back_end_path;
    back_end_path.header.frame_id = "world";
    back_end_path.header.stamp = ros::Time::now();
    
    geometry_msgs::PoseStamped p;
    for (double t=0.0; t<traj.getTotalDuration(); t+=0.03)
    {
        Eigen::Vector2d pos = traj.getPos(t);
        double yaw = traj.getAngle(t);
        p.pose.position.x = pos(0);
        p.pose.position.y = pos(1);
        p.pose.position.z = 0.0;
        p.pose.orientation.w = cos(yaw/2.0);
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = sin(yaw/2.0);
        back_end_path.poses.push_back(p);
    }
    Eigen::Vector2d pos = traj.getPos(traj.getTotalDuration());
    double yaw = traj.getAngle(traj.getTotalDuration());
    p.pose.position.x = pos(0);
    p.pose.position.y = pos(1);
    p.pose.position.z = 0.0;
    p.pose.orientation.w = cos(yaw/2.0);
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = sin(yaw/2.0);
    back_end_path.poses.push_back(p);

    se2_traj_pub.publish(back_end_path);
}


static double innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad)
{
    ALMTrajOpt& obj = *(ALMTrajOpt*) ptrObj;

    // get x
    const double& tau = x(0);
    double& gradtau = grad(0);
    Eigen::Map<const Eigen::MatrixXd> Pxy(x.data()+obj.dim_T, 2, obj.piece_xy-1);
    Eigen::Map<Eigen::MatrixXd> gradPxy(grad.data()+obj.dim_T, 2, obj.piece_xy-1);
    Eigen::Map<const Eigen::MatrixXd> Pyaw(x.data()+obj.dim_T+2*(obj.piece_xy-1), 1, obj.piece_yaw-1);
    Eigen::Map<Eigen::MatrixXd> gradPyaw(grad.data()+obj.dim_T+2*(obj.piece_xy-1), 1, obj.piece_yaw-1);

    // get T from tau, generate MINCO trajector
    Eigen::VectorXd Txy, Tyaw;
    Txy.resize(obj.piece_xy);
    Tyaw.resize(obj.piece_yaw);
    obj.calTfromTau(tau, Txy);
    obj.calTfromTau(tau, Tyaw);
    obj.minco_se2.generate(obj.init_xy, obj.end_xy, Pxy, Txy, obj.init_yaw, obj.end_yaw, Pyaw, Tyaw);

    // get jerk cost with grad (C, T)
    double jerk_cost = 0;
    Eigen::MatrixXd gdCxy_jerk, gdCyaw_jerk;
    Eigen::VectorXd gdTxy_jerk, gdTyaw_jerk;
    obj.minco_se2.calJerkGradCT(gdCxy_jerk, gdTxy_jerk, gdCyaw_jerk, gdTyaw_jerk);
    jerk_cost = obj.minco_se2.getTrajJerkCost() * obj.scale_fx;

    // get grad (q, T) from (C, T)
    Eigen::MatrixXd gdCxy = gdCxy_jerk * obj.scale_fx;
    Eigen::VectorXd gdTxy = gdTxy_jerk * obj.scale_fx;
    Eigen::MatrixXd gdCyaw = gdCyaw_jerk * obj.scale_fx;
    Eigen::VectorXd gdTyaw = gdTyaw_jerk * obj.scale_fx;
    Eigen::MatrixXd gradPxy_temp;
    Eigen::MatrixXd gradPyaw_temp;
    obj.minco_se2.calGradCTtoQT(gdCxy, gdTxy, gradPxy_temp, gdCyaw, gdTyaw, gradPyaw_temp);
    gradPxy = gradPxy_temp;
    gradPyaw = gradPyaw_temp;

    // get tau cost with grad
    double tau_cost = obj.rho_T * obj.expC2(tau) * obj.scale_fx;
    double grad_Tsum = obj.rho_T * obj.scale_fx + gdTxy.sum() / obj.piece_xy + gdTyaw.sum() / obj.piece_yaw;
    gradtau = grad_Tsum * obj.getTtoTauGrad(tau);

    return jerk_cost;
}


static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                    const double fx, const double step, int k, int ls)
{
    return k > 1e3;
}
