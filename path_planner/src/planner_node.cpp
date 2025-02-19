#include "alm_traj_opt.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    ALMTrajOpt traj_opt;
    traj_opt.init(nh);

    ros::spin();

    return 0;
}