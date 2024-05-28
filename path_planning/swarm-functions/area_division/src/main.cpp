#include "lib/area_division.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <map>
#include <string>


nav_msgs::OccupancyGrid current_map;
bool map_received = false;

map<string, vector<int>> cps_positions = {
    {"robot1", {214, 2}}
    // {"robot2", {194, 5}}, 
    // {"robot3", {130,60}}
};



// void processMap(const ros::Publisher& robot1_pub, const ros::Publisher& R1_start_pos_pub) {
//     if (map_received) {
//         ROS_INFO("Processing received map...");

//         // Initialize map in area_division object
//         ad.initialize_map(current_map.info.width, current_map.info.height, current_map.data);
//         ad.initialize_cps(cps_positions);
//         ad.divide();

//         // Get the divided maps
//         nav_msgs::OccupancyGrid robot1_grid = ad.get_grid(current_map, "robot1");

//         // Publish the divided maps
//         robot1_pub.publish(robot1_grid);

//         // Access the coordinates of the robots
//         std::vector<int> coordinates_R1 = cps_positions["robot1"];

//         // Create geometry_msgs::Point messages for starting positions
//         geometry_msgs::Point point_R1;
//         point_R1.x = coordinates_R1[0];
//         point_R1.y = coordinates_R1[1];

//         // Publish the starting points
//         R1_start_pos_pub.publish(point_R1);

//         map_received = false; // Reset flag
//         ROS_INFO("Map processing complete.");
//     }
// }


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    cps_positions = {
        {"robot1", {214, 2}}
        // {"robot2", {194, 5}},
        // {"robot3", {130, 60}}
    };
    current_map = *msg;
    map_received = true;
    ROS_INFO("Map received: width=%d, height=%d", current_map.info.width, current_map.info.height);
    //processMap();
}

int main(int argc, char **argv) {

    ROS_INFO("First step...");
    ros::init(argc, argv, "area_division_node");
    ros::NodeHandle nh;
    ROS_INFO("Second step...");
    area_division ad;
    ROS_INFO("Third step...");
    // Initialize CPS positions
    nav_msgs::OccupancyGrid robot1_grid;
    ROS_INFO("Fourth step...");
    
    ROS_INFO("Fifth step...");
    // Access the coordinates of the robots
    std::vector<int> coordinates_R1 = cps_positions["robot1"];
    // std::vector<int> coordinates_R2 = cps_positions["robot2"];
    // std::vector<int> coordinates_R3 = cps_positions["robot3"];

    // Create geometry_msgs::Point messages for starting positions
    geometry_msgs::Point point_R1, point_R2, point_R3;
    point_R1.x = coordinates_R1[0];
    point_R1.y = coordinates_R1[1];
    // point_R2.x = coordinates_R2[0];
    // point_R2.y = coordinates_R2[1];
    // point_R3.x = coordinates_R3[0];
    // point_R3.y = coordinates_R3[1];
    ROS_INFO("Sixth step...");
    // Publishers for the divided maps
    ros::Publisher robot1_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot1_grid", 10);
    // ros::Publisher robot2_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot2_grid", 10);
    // ros::Publisher robot3_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot3_grid", 10);

    // Publishers for the starting points
    ros::Publisher R1_start_pos_pub = nh.advertise<geometry_msgs::Point>("R1_starting_pos", 10);
    // ros::Publisher R2_start_pos_pub = nh.advertise<geometry_msgs::Point>("R2_starting_pos", 1);
    // ros::Publisher R3_start_pos_pub = nh.advertise<geometry_msgs::Point>("R3_starting_pos", 1);
    ROS_INFO("Seventh step...");
    // Subscriber for the occupancy grid map
    ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);

    ros::Rate loop_rate(1); // 1 Hz
    while (ros::ok()) {
        if (map_received) {
            ROS_INFO("Processing received map...");
            ad.initialize_map(current_map.info.width, current_map.info.height, current_map.data);

            ad.initialize_cps(cps_positions);
            // Perform area division
            ad.divide();

            // Get the divided maps
            robot1_grid = ad.get_grid(current_map, "robot1");

            for (size_t i = 0; i < current_map.data.size(); ++i) {
            if (current_map.data[i] == -1) {
                robot1_grid.data[i] = 100;
            }
            }


            // nav_msgs::OccupancyGrid robot2_grid = ad.get_grid(current_map, "robot2");
            // nav_msgs::OccupancyGrid robot3_grid = ad.get_grid(current_map, "robot3");

            // Publish the divided maps
            robot1_pub.publish(robot1_grid);
            // robot2_pub.publish(robot2_grid);
            // robot3_pub.publish(robot3_grid);

            // Publish the starting points
            R1_start_pos_pub.publish(point_R1);
            // R2_start_pos_pub.publish(point_R2);
            // R3_start_pos_pub.publish(point_R3);

            map_received = false; // Reset flag
            ROS_INFO("Map processing complete.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


            // std::vector<std::pair<int, int>> zero_indices = findZeroIndices(current_map);

            // // Logging zero indices
            // ROS_INFO("Zero value indices:");
            // for (const auto& idx : zero_indices) {
            //     ROS_INFO("(%d, %d)", idx.first, idx.second);
            // }
