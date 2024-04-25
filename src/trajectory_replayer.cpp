#include "amr_trajectory_viz/trajectory_replayer.h"

TrajectoryReplayer::TrajectoryReplayer(const ros::NodeHandle& node_handle) : nh_(node_handle) {
    std::string trajectory_file = load_trajectory_file();
    
    trajectory_.readFromCSV(trajectory_file);

    std::cout << "Successfully loaded trajectory from file: " << trajectory_file << std::endl;
    std::cout << "Loaded " << trajectory_.size() << " waypoints." << std::endl;

    std_msgs::ColorRGBA color;
    color.r = ros::param::param<float>("replayer/path_color/r", 0.0);
    color.g = ros::param::param<float>("replayer/path_color/g", 0.0);
    color.b = ros::param::param<float>("replayer/path_color/b", 1.0);
    color.a = ros::param::param<float>("replayer/path_color/a", 1.0);
    marker_array_ = trajectory_.toMarkerArray(color);

    std::string marker_topic_name;
    ros::param::param<std::string>("replayer/marker_topic_name",
                                    marker_topic_name, 
                                   "cached_trajectory_marker_array");
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_name, 5);
    std::cout << "Publishing trajectory to topic: " << marker_topic_name << std::endl;
}

void TrajectoryReplayer::run() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        marker_pub_.publish(marker_array_);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

std::string TrajectoryReplayer::load_trajectory_file() {
    std::string trajectory_file;
    nh_.param<std::string>("trajectory_csv", trajectory_file, "default.csv");
    boost::filesystem::path path(trajectory_file);

    if (path.is_relative()) {
        std::string package_path = ros::package::getPath("amr_trajectory_viz");
        if (!package_path.empty()) {
            path = boost::filesystem::path(package_path) / "logs" / path;
        } else {
            ROS_ERROR("Package not found.");
            exit(1);
        }
    }

    return path.string();
}
