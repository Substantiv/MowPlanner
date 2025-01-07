#include "coverage_path_planner.h"
#include "alm_traj_opt.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    CoveragePathPlanner path_planner;
    ALMTrajOpt traj_opt;
    path_planner.init(nh);
    traj_opt.init(nh);

    ros::spin();

    return 0;
}