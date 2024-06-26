#include "lib/area_division.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include <string>

// Global variables to store the current map and a flag to check if the map is received
nav_msgs::OccupancyGrid current_map;
bool map_received = false;

// Callback function for receiving the map
/**
 * @brief Callback function for receiving the occupancy grid map.
 * 
 * This function is called whenever a new occupancy grid message is received. It updates the
 * global variable with the latest map data and sets the flag indicating that a map has been received.
 * 
 * @param msg The incoming occupancy grid message.
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    current_map = *msg;
    map_received = true;
    ROS_INFO("Map received: width=%d, height=%d", current_map.info.width, current_map.info.height);
}


/**
 * @brief Get the transformation to the grid origin.
 * 
 * This function returns the transformation from the map frame to the grid's origin based on the
 * origin information provided in the occupancy grid message.
 * 
 * @param grid The occupancy grid message.
 * @return The transformation to the grid origin.
 */
tf::Transform getTransformToGridOrigin(const nav_msgs::OccupancyGrid& grid) {
    tf::Transform transform;

    // Set the translation to the position of the origin
    transform.setOrigin(tf::Vector3(grid.info.origin.position.x,
                                    grid.info.origin.position.y,
                                    grid.info.origin.position.z));

    // Set the orientation from the quaternion provided in the grid's origin
    tf::Quaternion q;
    tf::quaternionMsgToTF(grid.info.origin.orientation, q);
    transform.setRotation(q);

    return transform;
}


/**
 * @brief Get the robot's position in the map frame.
 * 
 * This function attempts to retrieve the robot's position in the map frame by first transforming
 * the robot's position from its base frame to the odom frame and then from the odom frame to the map frame.
 * 
 * @param listener The transform listener to use for retrieving the transformations.
 * @param robot_frame The frame ID of the robot's base link.
 * @param position The position of the robot in the map frame.
 * @return True if the position was successfully retrieved, otherwise false.
 */
bool getRobotPosition(tf::TransformListener &listener, const std::string &robot_frame, geometry_msgs::Point &position) {
    tf::StampedTransform transform_base_to_odom, transform_odom_to_map;
    try {
        // First check if the robot frame directly transforms to the odom frame
        listener.waitForTransform("odom", robot_frame, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("odom", robot_frame, ros::Time(0), transform_base_to_odom);

        // Look up the transform from the odom frame to the map frame
        listener.waitForTransform("map", "odom", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("map", "odom", ros::Time(0), transform_odom_to_map);

        // Combine the transformations to get the robot's position in the map frame
        tf::Transform transform_combined = transform_odom_to_map * transform_base_to_odom;

        position.x = transform_combined.getOrigin().x();
        position.y = transform_combined.getOrigin().y();
        position.z = transform_combined.getOrigin().z();
        return true;
    } catch (tf::TransformException &ex) {
        ROS_ERROR("Could not get transform for %s: %s", robot_frame.c_str(), ex.what());
        return false;
    }
}

/**
 * @brief Get the frame IDs of all detected robots.
 * 
 * This function retrieves the frame IDs of all detected robots by looking for frames containing
 * the string "base_link".
 * 
 * @param listener The transform listener to use for retrieving the frame IDs.
 * @return A vector of frame IDs corresponding to the detected robots.
 */
std::vector<std::string> getRobotFrames(tf::TransformListener &listener) {
    std::vector<std::string> robot_frames;
    std::vector<std::string> all_frames;
    
    listener.getFrameStrings(all_frames);

    ROS_INFO("Available frames:");
    for (const auto& frame : all_frames) {
        ROS_INFO("%s", frame.c_str());
        if (frame.find("base_link") != std::string::npos) {
            robot_frames.push_back(frame);
        }
    }
    
    return robot_frames;
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "area_division_node");
    ros::NodeHandle nh;

    // // Get the number of robots from the parameter server or default to 1
    // int num_robots;
    // nh.param("num_robots", num_robots, 1);
    // Not needed since we have num_robots_detected now

    // Initialize area division object
    area_division ad;

    // Create a TransformListener
    tf::TransformListener tf_listener;

    // Retry mechanism to get robot frames
    std::vector<std::string> robot_frames;
    for (int i = 0; i < 5; ++i) { // Retry up to 5 times
        ROS_INFO("Attempting to get robot frames, try %d", i + 1);
        robot_frames = getRobotFrames(tf_listener);
        if (!robot_frames.empty()) {
            break;
        }
        ros::Duration(2.0).sleep(); // Sleep for 2 seconds before retrying
    }

    int num_robots_detected = robot_frames.size();

    if (num_robots_detected == 0) {
        ROS_ERROR("No robot frames found.");
        return 1;
    }

    // Create publishers for each robot
    std::map<std::string, ros::Publisher> robot_pubs;
    std::map<std::string, ros::Publisher> start_pos_pubs;
    for (int i = 0; i < num_robots_detected; i++) {
        std::string robot_name = "robot" + std::to_string(i + 1);

    
        std::string grid_topic = "/" + robot_name + "_grid";
        std::string start_pos_topic = "/" + robot_name + "_starting_pos";

        // Create publishers with the new topic names
        ros::Publisher robot_pub = nh.advertise<nav_msgs::OccupancyGrid>(grid_topic, 10);
        ros::Publisher start_pos_pub = nh.advertise<geometry_msgs::Point>(start_pos_topic, 1);

        robot_pubs[robot_name] = robot_pub;
        start_pos_pubs[robot_name] = start_pos_pub;
    }

    // Subscriber for the occupancy grid map
    ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);

    ros::Rate loop_rate(0.1); // 1 Hz
    while (ros::ok()) {
        if (map_received) {
            ROS_INFO("Processing received map...");

            // Initialize the map in the area_division object
            ad.initialize_map(current_map.info.width, current_map.info.height, current_map.data);

            // Calculate the inverse transform from the map to the grid origin
            tf::Transform gridOriginTransform = getTransformToGridOrigin(current_map);
            tf::Transform inverseTransform = gridOriginTransform.inverse();

            // Get robot positions using tf
            std::map<std::string, std::vector<int>> updated_cps_positions;
            for (const auto& robot_frame : robot_frames) {
                std::string robot_name = robot_frame.substr(0, robot_frame.find('_'));
                geometry_msgs::Point position;
                if (getRobotPosition(tf_listener, robot_frame, position)) {
                    tf::Vector3 positionVector(position.x, position.y, position.z);
                    tf::Vector3 transformedPosition = inverseTransform * positionVector;

                    updated_cps_positions[robot_name] = {
                        std::max(0, static_cast<int>(transformedPosition.x())),
                        std::max(0, static_cast<int>(transformedPosition.y()))
                    };
                } else {
                    ROS_ERROR("Failed to get position for %s", robot_name.c_str());
                }
            }
            ad.initialize_cps(updated_cps_positions);
            ROS_INFO("Before division...");
            // Perform area division
            ad.divide();
            ROS_INFO("Post division...");

            // Create geometry_msgs::Point messages for starting positions and publish divided maps
            for (const auto& robot_frame : robot_frames) {
                std::string robot_name = robot_frame.substr(0, robot_frame.find('_'));
                int i=0;
                // Get the divided map for the robot
                nav_msgs::OccupancyGrid robot_grid = ad.get_grid(current_map, robot_name);

                // Ensure obstacles are marked
                for (size_t j = 0; j < current_map.data.size(); ++j) {
                    if (current_map.data[j] == -1) {
                        robot_grid.data[j] = 100;
                    }
                }
                std::string robot_name_publish = "robot" + std::to_string(i + 1);
                // Publish the divided map
                robot_pubs[robot_name_publish].publish(robot_grid);

                // Create and publish starting point
                std::vector<int> coordinates = updated_cps_positions[robot_name];
                geometry_msgs::Point start_point;
                start_point.x = coordinates[0];
                start_point.y = coordinates[1];
                start_pos_pubs[robot_name_publish].publish(start_point);
                i+=1;
            }

            map_received = false; // Reset the flag
            ROS_INFO("Map processing complete.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
