#include "lib/area_division.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <map>
#include <string>

// Global variables to store the current map and a flag to check if the map is received
nav_msgs::OccupancyGrid current_map;
bool map_received = false;

// Define the positions for the robots (this can be dynamic)
std::map<std::string, std::vector<int>> cps_positions = {
    {"robot1", {214, 2}}
    // {"robot2", {214, 5}}, 
    // {"robot3", {220,9}}
};

// Callback function for receiving the map
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    current_map = *msg;
    map_received = true;
    ROS_INFO("Map received: width=%d, height=%d", current_map.info.width, current_map.info.height);
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "area_division_node");
    ros::NodeHandle nh;

    // Get the number of robots from the parameter server or default to 1
    int num_robots;
    nh.param("num_robots", num_robots, 1);

    // Initialize area division object
    area_division ad;

    // Create publishers for each robot
    std::map<std::string, ros::Publisher> robot_pubs;
    std::map<std::string, ros::Publisher> start_pos_pubs;

    for (int i = 0; i < num_robots; i++) {
        std::string robot_name = "robot" + std::to_string(i + 1);

        ros::Publisher robot_pub = nh.advertise<nav_msgs::OccupancyGrid>(robot_name + "_grid", 10);
        ros::Publisher start_pos_pub = nh.advertise<geometry_msgs::Point>(robot_name + "_starting_pos", 1);

        robot_pubs[robot_name] = robot_pub;
        start_pos_pubs[robot_name] = start_pos_pub;
    }

    // Subscriber for the occupancy grid map
    ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);

    ros::Rate loop_rate(1); // 1 Hz
    while (ros::ok()) {
        if (map_received) {
            ROS_INFO("Processing received map...");

            // Initialize the map in the area_division object
            ad.initialize_map(current_map.info.width, current_map.info.height, current_map.data);

            // Initialize CPS positions (ensure it's updated based on the number of robots)
            std::map<std::string, std::vector<int>> updated_cps_positions;
            for (int i = 0; i < num_robots; i++) {
                std::string robot_name = "robot" + std::to_string(i + 1);
                updated_cps_positions[robot_name] = cps_positions[robot_name];
            }
            ad.initialize_cps(updated_cps_positions);

            // Perform area division
            ad.divide();

            // Create geometry_msgs::Point messages for starting positions and publish divided maps
            for (int i = 0; i < num_robots; i++) {
                std::string robot_name = "robot" + std::to_string(i + 1);
                
                // Get the divided map for the robot
                nav_msgs::OccupancyGrid robot_grid = ad.get_grid(current_map, robot_name);

                // Ensure obstacles are marked
                for (size_t j = 0; j < current_map.data.size(); ++j) {
                    if (current_map.data[j] == -1) {
                        robot_grid.data[j] = 100;
                    }
                }

                // Publish the divided map
                robot_pubs[robot_name].publish(robot_grid);

                // Create and publish starting point
                std::vector<int> coordinates = updated_cps_positions[robot_name];
                geometry_msgs::Point start_point;
                start_point.x = coordinates[0];
                start_point.y = coordinates[1];
                start_pos_pubs[robot_name].publish(start_point);
            }

            map_received = false; // Reset the flag
            ROS_INFO("Map processing complete.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
