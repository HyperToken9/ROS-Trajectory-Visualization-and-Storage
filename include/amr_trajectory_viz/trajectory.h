#include <tf/tf.h> 
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/filesystem.hpp>
#include <fstream>

/*
    Class stores the trajectory of the robot in the form of a vector 
    of Odometry messages.
*/
class Trajectory {

    public:
        /* Vector of Odometry messages */
        std::vector<nav_msgs::Odometry> trajectory_;

        /* Constructor */
        Trajectory();

        /* Converts the Odometry Message to Marker Array for Visualization */
        visualization_msgs::MarkerArray toMarkerArray(std_msgs::ColorRGBA color);

        /* Reads the trajectory from a CSV file */
        void readFromCSV(std::string file_path);
        
        /* Writes the trajectory to a CSV file */
        void writeToCSV(std::string file_path, double duration);

        /* 
            Adds an Odometry message to the trajectory 
            Precision is the distance threshold for adding the point 
            to the trajectory
        */
        void add(nav_msgs::Odometry odom_msg, double precision, double angular_precision);

        /* Returns size of trajectory */
        int size();

        /* Returns true if trajectory is empty */
        bool empty();

};
