#include "amr_trajectory_viz/trajectory_replayer.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_replay");
    ros::NodeHandle nh;

    TrajectoryReplayer replayer(nh);
    replayer.run();

    return 0;
}