

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp> 

#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include "amr_trajectory_viz/trajectory.h"

/*
    Class replays the trajectory of the robot.
    - Data is read from a CSV file
    - Publishes the trajectory as a MarkerArray 
*/

class TrajectoryReplayer {
public:

    /* Constructor */
    TrajectoryReplayer(const ros::NodeHandle& node_handle);

    /* Runs the node */
    void run();

private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    Trajectory trajectory_;
    visualization_msgs::MarkerArray marker_array_;

    /* 
        Loads the file for the correct absolute/ relative path 
        Reads CSV file and stores the trajectory
    */
    std::string load_trajectory_file();

};
