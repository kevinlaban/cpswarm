#include "coverage_path.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <mutex>
#include "your_package_name/PathPlanning.h"

std::mutex grid_mutex;
nav_msgs::OccupancyGrid latest_robot1_grid;
nav_msgs::OccupancyGrid latest_robot2_grid;
nav_msgs::OccupancyGrid latest_robot3_grid;
geometry_msgs::Point start_position1;
geometry_msgs::Point start_position2;
geometry_msgs::Point start_position3;

bool generate_path(geometry_msgs::Point start, const nav_msgs::OccupancyGrid& robot_occupancy_map, ros::Publisher& path_publisher, double resolution) {
    ros::NodeHandle nh;
    ROS_DEBUG("Starting at (%.2f,%.2f)", start.x, start.y);

    nav_msgs::OccupancyGrid area = robot_occupancy_map;
    ROS_INFO("Generate new coverage path...");

    // Calculate path resolution (distance between path legs)
    if (resolution == 0) {
        double fov, overlap;
        nh.param(this_node::getName() + "/fov", fov, 45.0);
        nh.param(this_node::getName() + "/overlap", overlap, 0.3);
        if (area.info.resolution == 0)
            nh.param(this_node::getName() + "/altitude", resolution, 50.0); // Set default altitude if not provided
        double width = 2 * resolution * tan(fov * M_PI / 180.0 / 2.0);
        resolution = width * (1 - overlap);
    }
    ROS_DEBUG("Map resolution: %.2f", resolution);

    spanning_tree tree;
    tree.initialize_graph(area, vertical);
    tree.construct();

    if (visualize)
        mst_publisher.publish(tree.get_tree());

    path.initialize_map(area, 0, vertical);
    path.initialize_tree(tree.get_mst_edges());
    if (!path.generate_path(start)) {
        ROS_WARN("Failed to generate path for the robot starting at (%.2f,%.2f)", start.x, start.y);
        return false;
    }
    if (turning_points)
        path.reduce();

    ROS_INFO("Visualize new coverage path...");
    path_publisher.publish(path.get_path());

    return true;
}

bool planPaths(your_package_name::PathPlanning::Request &req, your_package_name::PathPlanning::Response &res) {
    std::lock_guard<std::mutex> lock(grid_mutex);

    ros::NodeHandle nh;
    ros::Publisher path_publisher1 = nh.advertise<nav_msgs::Path>("coverage_path/path1", 1, true);
    ros::Publisher path_publisher2 = nh.advertise<nav_msgs::Path>("coverage_path/path2", 1, true);
    ros::Publisher path_publisher3 = nh.advertise<nav_msgs::Path>("coverage_path/path3", 1, true);

    if (latest_robot1_grid.info.width > 0 && latest_robot1_grid.info.height > 0) {
        if (!generate_path(start_position1, latest_robot1_grid, path_publisher1, req.resolution)) {
            res.success = false;
            res.message = "Failed to generate path for robot 1";
            return true;
        }
    }

    if (latest_robot2_grid.info.width > 0 && latest_robot2_grid.info.height > 0) {
        if (!generate_path(start_position2, latest_robot2_grid, path_publisher2, req.resolution)) {
            res.success = false;
            res.message = "Failed to generate path for robot 2";
            return true;
        }
    }

    if (latest_robot3_grid.info.width > 0 && latest_robot3_grid.info.height > 0) {
        if (!generate_path(start_position3, latest_robot3_grid, path_publisher3, req.resolution)) {
            res.success = false;
            res.message = "Failed to generate path for robot 3";
            return true;
        }
    }

    res.success = true;
    res.message = "Paths generated successfully";
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "coverage_path_node");
    ros::NodeHandle nh;

    ros::Subscriber robot1_sub = nh.subscribe("/robot1_grid", 10, robot1GridCallback);
    ros::Subscriber robot2_sub = nh.subscribe("/robot2_grid", 10, robot2GridCallback);
    ros::Subscriber robot3_sub = nh.subscribe("/robot3_grid", 10, robot3GridCallback);

    ros::Subscriber startPosSub_R1 = nh.subscribe("/robot1_starting_pos", 10, robot1StartPositionCallback);
    ros::Subscriber startPosSub_R2 = nh.subscribe("/robot2_starting_pos", 10, robot2StartPositionCallback);
    ros::Subscriber startPosSub_R3 = nh.subscribe("/robot3_starting_pos", 10, robot3StartPositionCallback);

    ros::ServiceServer service = nh.advertiseService("plan_paths", planPaths);

    ROS_INFO("Ready to plan paths.");

    ros::spin();

    return 0;
}
