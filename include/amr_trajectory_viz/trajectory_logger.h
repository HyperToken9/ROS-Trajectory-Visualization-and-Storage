
#include <fstream>
#include <boost/filesystem.hpp>

#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>

#include "amr_trajectory_viz/trajectory.h"
#include "amr_trajectory_viz/SaveTrajectory.h"

/*
    The class is responsible for logging the trajectory of the robot.
    - Data is stored in the Trajectory class.
    - Subscribes to the given odometry topic
    - Publishes the trajectory as a MarkerArray
    - Provides a service to save the trajectory to a CSV file
*/

class TrajectoryLogger {
public:
    
    /* Constructor */
    TrajectoryLogger(const ros::NodeHandle& nh);

    /* Runs the node */
    void run();

private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber odom_sub_;
    ros::ServiceServer save_service_;
    Trajectory trajectory_;
    /* Prevents duplicate and unneeded data */
    double precision_;
    double angular_precision_;

    /* 
        Callback for the odometry topic 
        Publishes the trajectory as a MarkerArray
    */
    void trajectoryLoggerCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    /*
        Service to save the trajectory to a CSV file
    
        Request: 
            - file_path: path to the file where the trajectory should be saved
            - duration: duration of the trajectory in seconds
    */
    bool saveTrajectoryService(amr_trajectory_viz::SaveTrajectory::Request &request,
                               amr_trajectory_viz::SaveTrajectory::Response &response);
};
