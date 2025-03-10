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

    coverage_path_sub = nh.subscribe<nav_msgs::Path>("/kino_astar/path", 10, &ALMTrajOpt::pathCallback, this);
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

    // Distribute init_xy(0.05) and end_xy(0.05) velocity to x and y based on yaw angle
    init_xy.col(1) << 0.05*cos(init_yaw(0)), 0.05*sin(init_yaw(0));
    end_xy.col(1) << 0.05*cos(end_yaw(0)), 0.05*sin(end_yaw(0));

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

    visSE2Traj(back_end_traj);
}


int ALMTrajOpt::optimizeTraj(const Eigen::MatrixXd &initStateXY, const Eigen::MatrixXd &endStateXY, const Eigen::MatrixXd &innerPtsXY, \
                            const Eigen::VectorXd &initYaw, const Eigen::VectorXd &endYaw, const Eigen::VectorXd &innerPtsYaw, \
                            const double & totalTime){

    piece_xy = innerPtsXY.cols() + 1;       // Number of segments in the xy trajectory
    piece_yaw = innerPtsYaw.size() + 1;     // Number of segments in the yaw trajectory                   
    init_xy = initStateXY;
    end_xy = endStateXY;
    init_yaw = initYaw.transpose();
    end_yaw = endYaw.transpose();
    minco_se2.reset(piece_xy, piece_yaw);

    // variable number: total_time + inner_xy_point + inner_yaw_point
    int variable_num = 1 + 2*(piece_xy-1) + (piece_yaw-1);
    // non-holonomic constraint: dx*sin(yaw) + dy*cos(yaw) = 0
    equal_num = piece_xy * (int_K + 1); // Discretize each segment into K parts, resulting in K+1 points per segment
    // dynamic feasibility constrains (longitude velocity, longitude acceleration, latitude acceleration, curvature)
    // user-defined constrains(attitude, terrain curvature)
    non_equal_num = piece_xy * (int_K + 1) * 6;

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

    // Allocate variables
    dim_T = 1;
    // Time optimization variable: total time
    double& tau = x(0);
    tau = logC2(totalTime);

    // xy trajectory optimization variables: intermediate XY trajectory points
    Eigen::Map<Eigen::MatrixXd> Pxy(x.data() + dim_T, 2, piece_xy - 1);
    Pxy = innerPtsXY;

    // yaw trajectory optimization variables: intermediate yaw trajectory points
    Eigen::Map<Eigen::MatrixXd> Pyaw(x.data() + dim_T + 2 * (piece_xy - 1), 1, piece_yaw - 1);
    Pyaw = innerPtsYaw.transpose();

    // lbfgs params
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = mem_size;                // Number of historical variables used for approximating the Hessian matrix
    lbfgs_params.past = past;                        // Length of the history for checking the convergence of the objective function
    lbfgs_params.g_epsilon = g_epsilon;              // Gradient stopping criterion. Stops iteration when the infinity norm of the gradient is below this value
    lbfgs_params.min_step = min_step;                // Minimum step size limit
    lbfgs_params.delta = delta;                      // Convergence criterion based on the change in objective value. If the change is smaller than this value, convergence is assumed
    lbfgs_params.max_iterations = inner_max_iter;    // Maximum number of iterations    
    double inner_cost;

    // begin PHR Augmented Lagrangian Method
    ros::Time start_time = ros::Time::now();
    int iter = 0;

    int ret_code;
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

    // get constrain cost with grad (C,T)
    double constrain_cost = 0.0;
    Eigen::MatrixXd gdCxy_constrain, gdCyaw_constrain;
    Eigen::VectorXd gdTxy_constrain, gdTyaw_constrain;
    obj.calConstrainCostGrad(constrain_cost, gdCxy_constrain, gdTxy_constrain, gdCyaw_constrain, gdTyaw_constrain);

    // get grad (q, T) from (C, T)
    // Eigen::MatrixXd gdCxy = gdCxy_jerk * obj.scale_fx + gdCxy_constrain;
    // Eigen::VectorXd gdTxy = gdTxy_jerk * obj.scale_fx + gdTxy_constrain;
    // Eigen::MatrixXd gdCyaw = gdCyaw_jerk * obj.scale_fx + gdCyaw_constrain;
    // Eigen::VectorXd gdTyaw = gdTyaw_jerk * obj.scale_fx + gdTyaw_constrain;
    Eigen::MatrixXd gdCxy = gdCxy_jerk * obj.scale_fx;
    Eigen::VectorXd gdTxy = gdTxy_jerk * obj.scale_fx;
    Eigen::MatrixXd gdCyaw = gdCyaw_jerk * obj.scale_fx;
    Eigen::VectorXd gdTyaw = gdTyaw_jerk * obj.scale_fx;
    Eigen::MatrixXd gradPxy_temp, gradPyaw_temp;
    obj.minco_se2.calGradCTtoQT(gdCxy, gdTxy, gradPxy_temp, gdCyaw, gdTyaw, gradPyaw_temp);
    gradPxy = gradPxy_temp;
    gradPyaw = gradPyaw_temp;

    // get tau cost with grad
    double tau_cost = obj.rho_T * obj.expC2(tau) * obj.scale_fx;
    double grad_Tsum = obj.rho_T * obj.scale_fx + gdTxy.sum() / obj.piece_xy + gdTyaw.sum() / obj.piece_yaw;
    gradtau = grad_Tsum * obj.getTtoTauGrad(tau);

    return jerk_cost + tau_cost;
}


void ALMTrajOpt::calConstrainCostGrad(double& cost,\
                                     Eigen::MatrixXd& gdCxy, Eigen::VectorXd &gdTxy, \
                                     Eigen::MatrixXd& gdCyaw, Eigen::VectorXd &gdTyaw)
{
    cost = 0.0;
    gdCxy.resize(6*piece_xy, 2);
    gdCxy.setZero();
    gdTxy.resize(piece_xy);
    gdTxy.setZero();
    gdCyaw.resize(piece_yaw, 1);
    gdCyaw.setZero();
    gdTyaw.resize(piece_yaw);
    gdTyaw.setZero();

    double step, alpha, omega;
    double s1, s2, s3, s4, s5;
    double s1_yaw, s2_yaw, s3_yaw, s4_yaw, s5_yaw;

    Eigen::Vector2d pos, vel, acc, jer;
    Eigen::Vector2d xb, yb;
    Eigen::Vector3d se2_pos;

    double          yaw, dyaw, d2yaw;
    double          cyaw, syaw, v_norm;
    double          lon_acc, lat_acc, curv_snorm, vx, wz, ax, ay;
    double          inv_cos_vphix, sin_phix, inv_cos_vphiy, sin_phiy, cos_xi, inv_cos_xi, sigma;

    Eigen::Vector2d grad_p = Eigen::Vector2d::Zero();
    Eigen::Vector2d grad_v = Eigen::Vector2d::Zero();
    Eigen::Vector2d grad_a = Eigen::Vector2d::Zero();
    Eigen::Vector3d grad_se2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_inv_cos_vphix, grad_sin_phix, grad_inv_cos_vphiy, grad_sin_phiy, grad_cos_xi, grad_inv_cos_xi, grad_sigma;
    double          gravity = 9.8;
    double          grad_yaw = 0.0;
    double          grad_dyaw = 0.0;
    double          grad_d2yaw = 0.0;
    double          aug_grad = 0.0;
    double          grad_vx2 = 0.0;
    double          grad_wz = 0.0;
    double          grad_ax = 0.0;
    double          grad_ay = 0.0;

    // beta matrix
    Eigen::Matrix<double, 6, 1> beta0_xy, beta1_xy, beta2_xy, beta3_xy;
    Eigen::Matrix<double, 6, 1> beta0_yaw, beta1_yaw, beta2_yaw, beta3_yaw;

    int equal_idx = 0;
    int non_equal_idx = 0;
    int constrain_idx = 0;
    int yaw_idx = 0;
    double base_time = 0.0;

    for(int i=0; i<piece_xy; i++) // i=1
    {
        const Eigen::Matrix<double, 6, 2> &c_xy = minco_se2.pos_minco.getCoeffs().block<6, 2>(i*6, 0);
        step = minco_se2.pos_minco.T1(i) / int_K;
        s1 = 0.0;

        for(int j=0; j<=int_K; j++) //j=8
        {
            alpha = 1. / int_K * j;

            grad_p.setZero();
            grad_v.setZero();
            grad_a.setZero();
            grad_yaw = 0.0;
            grad_dyaw = 0.0;
            grad_d2yaw = 0.0;
            grad_vx2 = 0.0;
            grad_wz = 0.0;
            grad_ax = 0.0;
            grad_ay = 0.0;
            grad_se2.setZero();

            // analyse xy
            s2 = s1 * s1;
            s3 = s2 * s1;
            s4 = s3 * s1;
            s5 = s4 * s1;
            beta0_xy << 1.0, s1, s2, s3, s4, s5;
            beta1_xy << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
            beta2_xy << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
            beta3_xy << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
            pos = c_xy.transpose() * beta0_xy;
            vel = c_xy.transpose() * beta1_xy;
            acc = c_xy.transpose() * beta2_xy;
            jer = c_xy.transpose() * beta3_xy;

            // analyse yaw 
            // TODO：这个base_time是干什么的
            double now_time = s1 +base_time;
            yaw_idx = int((now_time) / minco_se2.yaw_minco.T1(i));
            if (yaw_idx >= piece_yaw)
                yaw_idx = piece_yaw - 1;
            const Eigen::Matrix<double, 6, 1> &c_yaw = minco_se2.yaw_minco.getCoeffs().block<6, 1>(yaw_idx * 6, 0);
            s1_yaw = now_time - yaw_idx * minco_se2.yaw_minco.T1(i);
            s2_yaw = s1_yaw * s1_yaw;
            s3_yaw = s2_yaw * s1_yaw;
            s4_yaw = s2_yaw * s2_yaw;
            s5_yaw = s4_yaw * s1_yaw;
            beta0_yaw << 1.0, s1_yaw, s2_yaw, s3_yaw, s4_yaw, s5_yaw;
            beta1_yaw << 0.0, 1.0, 2.0 * s1_yaw, 3.0 * s2_yaw, 4.0 * s3_yaw, 5.0 * s4_yaw;
            beta2_yaw << 0.0, 0.0, 2.0, 6.0 * s1_yaw, 12.0 * s2_yaw, 20.0 * s3_yaw;
            beta3_yaw << 0.0, 0.0, 0.0, 6.0, 24.0 * s1_yaw, 60.0 * s2_yaw;
            yaw = c_yaw.transpose() * beta0_yaw;
            dyaw = c_yaw.transpose() * beta1_yaw;
            d2yaw = c_yaw.transpose() * beta2_yaw;

            // analyse complex variable(kinematics variable)
            se2_pos = Eigen::Vector3d(pos(0), pos(1), yaw);
            syaw = sin(yaw);
            cyaw = cos(yaw);
            v_norm = vel.norm();
            xb = Eigen::Vector2d(cyaw, syaw);
            yb = Eigen::Vector2d(-syaw, cyaw);
            lon_acc = acc.dot(xb);
            lat_acc = acc.dot(yb);

            // TODO: user-defined cost: surface variation
            if (j==0 || j==int_K)
                omega = 0.5 * rho_ter * step * scale_fx;
            else
                omega = rho_ter * step * scale_fx;
            double user_cost = omega * sigma * sigma;
            cost += user_cost;
            grad_se2 += omega * grad_sigma * sigma * 2.0;
            gdTxy(i) += user_cost / int_K;

            // non-holonomic
            double nonh_lambda = lambda[equal_idx];
            Eigen::Vector2d non_holonomic_yaw(syaw, -cyaw);
            hx[equal_idx] = vel.dot(non_holonomic_yaw) * scale_cx(constrain_idx);
            cost += getAugmentedCost(hx[equal_idx], nonh_lambda);
            double nonh_grad = getAugmentedGrad(hx[equal_idx], nonh_lambda) * scale_cx(constrain_idx);
            grad_v += nonh_grad * non_holonomic_yaw;
            grad_yaw += nonh_grad * vel.dot(xb);
            equal_idx++;
            constrain_idx++;
            
            // longitude velocity
            double v_mu = mu[non_equal_idx];
            gx[non_equal_idx] = (vx*vx - max_vel*max_vel) * scale_cx(constrain_idx);
            if (rho * gx[non_equal_idx] + v_mu > 0)
            {
                cost += getAugmentedCost(gx[non_equal_idx], v_mu);
                aug_grad = getAugmentedGrad(gx[non_equal_idx], v_mu) * scale_cx(constrain_idx);
                grad_vx2 += aug_grad;
            }
            else
            {
                cost += -0.5 * v_mu * v_mu / rho;
            }
            non_equal_idx++;
            constrain_idx++;

            // longitude acceleration
            double lona_mu = mu[non_equal_idx];
            gx[non_equal_idx] = (ax*ax - max_acc_lon*max_acc_lon) * scale_cx(constrain_idx);
            if (rho * gx[non_equal_idx] + lona_mu > 0)
            {
                cost += getAugmentedCost(gx[non_equal_idx], lona_mu);
                aug_grad = getAugmentedGrad(gx[non_equal_idx], lona_mu) * scale_cx(constrain_idx);
                grad_ax += aug_grad * 2.0 * ax;
            }
            else
            {
                cost += -0.5 * lona_mu * lona_mu / rho;
            }
            non_equal_idx++;
            constrain_idx++;

            // latitude acceleration
            double lata_mu = mu[non_equal_idx];
            gx[non_equal_idx] = (ay*ay - max_acc_lat*max_acc_lat) * scale_cx(constrain_idx);
            if (rho * gx[non_equal_idx] + lata_mu > 0)
            {
                cost += getAugmentedCost(gx[non_equal_idx], lata_mu);
                aug_grad = getAugmentedGrad(gx[non_equal_idx], lata_mu) * scale_cx(constrain_idx);
                grad_ay += aug_grad * 2.0 * ay;
            }
            else
            {
                cost += -0.5 * lata_mu * lata_mu / rho;
            }
            non_equal_idx++;
            constrain_idx++;

            // curvature
            double curv_mu = mu[non_equal_idx];
            if (use_scaling)
                gx[non_equal_idx] = (curv_snorm - max_kap*max_kap) * scale_cx(constrain_idx);
            else
                gx[non_equal_idx] = (curv_snorm - max_kap*max_kap) * cur_scale;
            if (rho * gx[non_equal_idx] + curv_mu > 0)
            {
                double denominator = 1.0 / (vx*vx + delta_sigl);
                cost += getAugmentedCost(gx[non_equal_idx], curv_mu);
                if (use_scaling)
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], curv_mu) * scale_cx(constrain_idx);
                else
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], curv_mu) * cur_scale;
                grad_wz += aug_grad * denominator * 2.0 * wz;
                grad_vx2 -= aug_grad * curv_snorm * denominator;
            }
            else
            {
                cost += -0.5 * curv_mu * curv_mu / rho;
            }
            non_equal_idx++;
            constrain_idx++;

            // attitude
            double att_mu = mu[non_equal_idx];
            gx[non_equal_idx] = (min_cxi - cos_xi) * scale_cx(constrain_idx);
            if (rho * gx[non_equal_idx] + att_mu > 0)
            {
                cost += getAugmentedCost(gx[non_equal_idx], att_mu);
                grad_se2 -= getAugmentedGrad(gx[non_equal_idx], att_mu) * grad_cos_xi * scale_cx(constrain_idx);
            }
            else
            {
                cost += -0.5 * att_mu * att_mu / rho;
            }
            non_equal_idx++;
            constrain_idx++;

            // TODO: user-defined: surface variation
            double sig_mu = mu[non_equal_idx];
            if (use_scaling)
                gx[non_equal_idx] = (sigma - max_sig) * scale_cx(constrain_idx);
            else
                gx[non_equal_idx] = (sigma - max_sig) * sig_scale;
            if (rho * gx[non_equal_idx] + sig_mu > 0)
            {
                cost += getAugmentedCost(gx[non_equal_idx], sig_mu);
                if (use_scaling)
                    grad_se2 += getAugmentedGrad(gx[non_equal_idx], sig_mu) * grad_sigma * scale_cx(constrain_idx);
                else
                    grad_se2 += getAugmentedGrad(gx[non_equal_idx], sig_mu) * grad_sigma * sig_scale;
            }
            else
            {
                cost += -0.5 * sig_mu * sig_mu / rho;
            }
            non_equal_idx++;
            constrain_idx++;

            // process with vx, wz, ax
            grad_v += grad_vx2 * inv_cos_vphix * inv_cos_vphix * 2.0 * vel;
            grad_se2 += grad_vx2 * v_norm * v_norm * 2.0 * inv_cos_vphix * grad_inv_cos_vphix;
            
            grad_dyaw += grad_wz * inv_cos_xi;
            grad_se2 += grad_wz * dyaw * grad_inv_cos_xi;

            grad_a += grad_ax * inv_cos_vphix * xb;
            grad_yaw += grad_ax * inv_cos_vphix * lat_acc;
            grad_se2 += grad_ax * (gravity * grad_sin_phix + grad_inv_cos_vphix * lon_acc);

            grad_a += grad_ay * inv_cos_vphiy * yb;
            grad_yaw -= grad_ay * inv_cos_vphiy * lon_acc;
            grad_se2 += grad_ay * (gravity * grad_sin_phiy + grad_inv_cos_vphiy * lat_acc);

            grad_p += grad_se2.head(2);
            grad_yaw += grad_se2(2);

            // add all grad into C,T
            // note that xy = Cxy*β(j/K*T_xy), yaw = Cyaw*β(i*T_xy+j/K*T_xy-yaw_idx*T_yaw)
            // ∂p/∂Cxy, ∂v/∂Cxy, ∂a/∂Cxy
            gdCxy.block<6, 2>(i * 6, 0) += (beta0_xy * grad_p.transpose() + beta1_xy * grad_v.transpose() + beta2_xy * grad_a.transpose());
            // ∂p/∂Txy, ∂v/∂Txy, ∂a/∂Txy
            gdTxy(i) += (grad_p.dot(vel) + grad_v.dot(acc) + grad_a.dot(jer) ) * alpha;
            // ∂yaw/∂Cyaw, ∂dyaw/∂Cyaw, ∂d2yaw/∂Cyaw
            // TODO:这里有bug
            gdCyaw.block<6, 1>(yaw_idx * 6, 0) += (beta0_yaw * grad_yaw + beta1_yaw * grad_dyaw + beta2_yaw * grad_d2yaw);
            // ∂yaw/∂Tyaw, ∂dyaw/∂Tyaw, ∂d2yaw/∂Tyaw
            gdTyaw(yaw_idx) += -(grad_yaw * dyaw + grad_dyaw * d2yaw) * yaw_idx;
            // ∂yaw/∂Txy, ∂dyaw/∂Txy, ∂d2yaw/∂Txy
            gdTxy(i) += (grad_yaw * dyaw + grad_dyaw * d2yaw) * (alpha+i);

            s1 += step;
        }
        base_time += minco_se2.pos_minco.T1(i);
    }
}


static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                    const double fx, const double step, int k, int ls)
{
    return k > 1e3;
}
