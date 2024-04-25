#include "ros/ros.h"
#include "amr_trajectory_viz/trajectory_logger.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_logger");
    ros::NodeHandle nh;

    TrajectoryLogger logger(nh);
    logger.run();

    return 0;
}