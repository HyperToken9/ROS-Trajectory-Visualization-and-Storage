#include <amr_trajectory_viz/trajectory.h>


visualization_msgs::MarkerArray Trajectory::toMarkerArray(std_msgs::ColorRGBA color)
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    // Set up the marker properties
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory_arrows";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the arrow
    marker.scale.x = 0.4;  
    marker.scale.y = 0.04; 
    marker.scale.z = 0.04;

    // Set the color of the arrow
    marker.color = color;

    for (int i = 0; i < trajectory_.size(); ++i) {
        
        // Set the ID of the marker
        marker.id = i;
    
        // Set the position and orientation
        marker.pose.position = trajectory_[i].pose.pose.position;
        marker.pose.orientation = trajectory_[i].pose.pose.orientation;

        // Add the populated marker to the marker array
        marker_array.markers.push_back(marker);
    }

    return marker_array;
}

void Trajectory::readFromCSV(std::string file_path) {

    std::ifstream csv_file(file_path);
    std::string line;
    
    std::getline(csv_file, line);

    std::istringstream ss(line);
    std::string token;
    int count = 0;
    
    while (std::getline(ss, token, ',')) {
        count++;
    }
    
    if (count != 8){
        std::cout << "Invalid CSV file format. Expected 8 columns, got " << count << std::endl;
        exit(0);
    }

    while (std::getline(csv_file, line)) {
        std::istringstream ss(line);
        std::string token;
        nav_msgs::Odometry odom_msg;
        int i = 0;
        
        while (std::getline(ss, token, ',')) {
            switch (i) {
                case 0:
                    odom_msg.header.stamp = ros::Time(std::stod(token));
                    break;
                case 1:
                    odom_msg.pose.pose.position.x = std::stod(token);
                    break;
                case 2:
                    odom_msg.pose.pose.position.y = std::stod(token);
                    break;
                case 3:
                    odom_msg.pose.pose.position.z = std::stod(token);
                    break;
                case 4:
                    odom_msg.pose.pose.orientation.x = std::stod(token);
                    break;
                case 5:
                    odom_msg.pose.pose.orientation.y = std::stod(token);
                    break;
                case 6:
                    odom_msg.pose.pose.orientation.z = std::stod(token);
                    break;
                case 7:
                    odom_msg.pose.pose.orientation.w = std::stod(token);
                    break;
            }
            i++;
        }
        
        this->add(odom_msg, 0, 0);
    }
}

void Trajectory::writeToCSV(std::string file_path, double duration){
    
    std::ofstream csv_file; 
    csv_file.open(file_path);

    if (!csv_file.is_open())
    {
        ROS_ERROR("Could not open file %s", file_path.c_str());
        return;
    }

    ros::Time now = ros::Time::now();

    // Write CSV header
    csv_file << "Timestamp,Position X,Position Y,Position Z,Orientation X,Orientation Y,Orientation Z,Orientation W\n";

    for (const nav_msgs::Odometry& odom : trajectory_) {

        if ((now - odom.header.stamp).toSec() <= duration) {
        
            csv_file << odom.header.stamp << ","
                     << odom.pose.pose.position.x << ","
                     << odom.pose.pose.position.y << ","
                     << odom.pose.pose.position.z << ","
                     << odom.pose.pose.orientation.x << ","
                     << odom.pose.pose.orientation.y << ","
                     << odom.pose.pose.orientation.z << ","
                     << odom.pose.pose.orientation.w << "\n";
        }
    }

    csv_file.close();

}

void Trajectory::add(nav_msgs::Odometry odom_msg, double precision, double angular_precision) {
    if (trajectory_.size() == 0) {
        trajectory_.push_back(odom_msg);
    } else {
        const nav_msgs::Odometry& last_odom = trajectory_.back();
        
        // Calculate linear distance
        double dx = last_odom.pose.pose.position.x - odom_msg.pose.pose.position.x;
        double dy = last_odom.pose.pose.position.y - odom_msg.pose.pose.position.y;
        double dz = last_odom.pose.pose.position.z - odom_msg.pose.pose.position.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Calculate angular difference
        tf::Quaternion q_last, q_current;
        tf::quaternionMsgToTF(last_odom.pose.pose.orientation, q_last);
        tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, q_current);
        double angular_difference = q_last.angleShortestPath(q_current);

        if (distance > precision || angular_difference > angular_precision) {
            trajectory_.push_back(odom_msg);
        }
    }
}

int Trajectory::size() {
    return trajectory_.size();
}

bool Trajectory::empty() {
    return trajectory_.empty();
}

Trajectory::Trajectory() {

}