#include "amr_trajectory_viz/trajectory_logger.h"

TrajectoryLogger::TrajectoryLogger(const ros::NodeHandle& nh)
    : nh_(nh) 
{

    ros::param::param<double>("logger/precision",
                              precision_, 
                              0.1);

    std::string marker_topic_name;
    ros::param::param<std::string>("logger/marker_topic_name",
                                    marker_topic_name, 
                                    "trajectory_marker_array");
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_name, 5);

    std::string odom_topic_name;
    ros::param::param<std::string>("logger/odom_topic_name",
                                    odom_topic_name, 
                                    "/odom");
    odom_sub_ = nh_.subscribe(odom_topic_name, 100, &TrajectoryLogger::trajectoryLoggerCallback, this);
    
    std::string service_name;
    ros::param::param<std::string>("logger/service_name",
                                    service_name, 
                                    "log_trajectory");
    save_service_ = nh_.advertiseService(service_name, &TrajectoryLogger::saveTrajectoryService, this);
}

void TrajectoryLogger::run() {
    ros::spin();
}

void TrajectoryLogger::trajectoryLoggerCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    trajectory_.add(*odom_msg, precision_, angular_precision_);

    std_msgs::ColorRGBA color;
    color.r = ros::param::param<float>("logger/path_color/r", 0.0);
    color.g = ros::param::param<float>("logger/path_color/g", 1.0);
    color.b = ros::param::param<float>("logger/path_color/b", 0.0);
    color.a = ros::param::param<float>("logger/path_color/a", 1.0);
    visualization_msgs::MarkerArray marker_array = trajectory_.toMarkerArray(color);
    marker_pub_.publish(marker_array);
}

bool TrajectoryLogger::saveTrajectoryService(amr_trajectory_viz::SaveTrajectory::Request &request,
                                             amr_trajectory_viz::SaveTrajectory::Response &response) {
    
    std::string package_path = ros::package::getPath("amr_trajectory_viz");
    
    boost::filesystem::path logs_path(package_path);
    logs_path /= "logs"; 

    if (!boost::filesystem::exists(logs_path)) {
        boost::filesystem::create_directories(logs_path);
    }

    boost::filesystem::path file_path = logs_path / (request.filename + ".csv");

    trajectory_.writeToCSV(file_path.string(), request.duration);

    return true;
}