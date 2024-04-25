# Trajectory Visualization and Storage

## Overview
This ROS package facilitates live tracking and storage of paths taken by your robot, allowing for high customization of path precision. It is also capable of replaying stored paths for visualization purposes.

## Features
This package includes several key features:
- Visualization of trajectories in RViz.
- Storage of trajectory data in CSV format.
- Automated collection and publication of trajectory data.
- A service-based interface for specifying time durations when saving trajectory data.

## Installation
Follow these steps to install the package in a ROS environment:
```bash
cd ~/catkin_ws/src
git clone github.com/HyperToken9/ROS-Trajectory-Visualization-and-Storage.git
cd ..
catkin build
source devel/setup.bash
```

## Package Contents

### `logger_node`
This node records and publishes trajectory data and provides a service for data saving:
- **trajectory_marker_array** (Publisher): Publishes the path taken by the bot with respect to a frame (default frame: /odom).
- **log_trajectory** (Service): 
    - Parameters: filename, duration
    - Description: Logs the path within the specified duration to a CSV file.

### `replayer_node`
This node reads and publishes saved trajectory data for visualization:
- **cached_trajectory_marker_array** (Publisher): Reads a saved CSV and publishes it as a marker array.

## Usage
Examples of how to run the nodes and services provided by the package:

### Running the Nodes
```bash
# To run the trajectory publisher and saver node
roslaunch amr_trajectory_viz log_trajectory.launch

# To run the trajectory reader and publisher node
rosrun amr_trajectory_viz replay_trajectory.launch
```

### Using the Services
```bash
# To save trajectory data
rosservice call /log_trajectory "filename: 'session1'
duration: 60"

# OR
rosservice call /log_trajectory session1 60
```

## Configuration
Adjust settings in `logger.yaml` and `replayer.yaml` to tune the nodes according to your application needs.

## Node Details
Node details are generated based on the above package contents.

## Algorithms
The primary algorithm for this package optimizes how trajectory data is stored to avoid excessive memory usage due to continuous publication of odometry messages. By setting precision parameters for linear and angular changes, new odometry messages are added to the trajectory list only when significant changes occur. This method also preserves the temporal pattern of the path, capturing moments when the robot remains stationary.

### Pseudo C++ Code Example
```cpp
if (trajectory_.empty()) {
    trajectory_.push_back(odom_msg);
} else {
    const nav_msgs::Odometry& last_odom = trajectory_.back();
    
    double dx = last_odom.pose.pose.position.x - odom_msg.pose.pose.position.x;
    double dy = last_odom.pose.pose.position.y - odom_msg.pose.pose.position.y;
    double dz = last_odom.pose.pose.position.z - odom_msg.pose.pose.position.z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);

    tf::Quaternion q_last, q_current;
    tf::quaternionMsgToTF(last_odom.pose.pose.orientation, q_last);
    tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, q_current);
    double angular_difference = q_last.angleShortestPath(q_current);

    if (distance > precision || angular_difference > angular_precision) {
        trajectory_.push_back(odom_msg);
    }
}
```

## Author
**Nathan Adrian Saldanha**