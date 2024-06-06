#include "coverage_path.h"
#include <mutex>
#include <iostream>
#include <vector>
#include <cmath> // For std::ceil
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Trigger.h>
#include <coverage_path/GeneratePath.h>

std::mutex grid_mutex;  // Mutex for thread-safe access to grid variables
 

//Publishers
ros::Publisher path_publisher1;
ros::Publisher path_publisher2;
ros::Publisher path_publisher3;

// Shared variable to store the latest grid data
nav_msgs::OccupancyGrid latest_robot1_grid;
nav_msgs::OccupancyGrid latest_robot2_grid;
nav_msgs::OccupancyGrid latest_robot3_grid;

//Shared variable to store starting positions
geometry_msgs::Point start_position1;
geometry_msgs::Point start_position2;
geometry_msgs::Point start_position3;

nav_msgs::OccupancyGrid parseGrid(const nav_msgs::OccupancyGrid& originalGrid, double desiredResolution) {
    double originalResolution = originalGrid.info.resolution;
    
    if (desiredResolution < originalResolution) {
        throw std::invalid_argument("Desired resolution must be greater than or equal to the original resolution.");
    }
    
    double scale_factor = desiredResolution / originalResolution;
    int roundedFactor = std::round(scale_factor);
    
    // Calculate the new resolution
    double adjustedResolution = originalResolution * roundedFactor;
    
    nav_msgs::OccupancyGrid downsizedGrid;
    downsizedGrid.info = originalGrid.info;
    downsizedGrid.info.width = std::ceil(originalGrid.info.width / roundedFactor);
    downsizedGrid.info.height = std::ceil(originalGrid.info.height / roundedFactor);
    downsizedGrid.info.resolution = adjustedResolution;
    downsizedGrid.data.resize(downsizedGrid.info.width * downsizedGrid.info.height);
    
    for (int y = 0; y < downsizedGrid.info.height; ++y) {
        for (int x = 0; x < downsizedGrid.info.width; ++x) {
            int originalXStart = x * roundedFactor;
            int originalYStart = y * roundedFactor;
            int originalXEnd = std::min(static_cast<int>(originalXStart + roundedFactor), static_cast<int>(originalGrid.info.width));
            int originalYEnd = std::min(static_cast<int>(originalYStart + roundedFactor), static_cast<int>(originalGrid.info.height));
            int sum = 0, count = 0, foundUnknown = false;
            for (int i = originalYStart; i < originalYEnd; ++i) {
                for (int j = originalXStart; j < originalXEnd; ++j) {
                    int index = i * originalGrid.info.width + j;
                    if (originalGrid.data[index] == -1) {
                        foundUnknown = true;
                        break;
                    }
                    sum += originalGrid.data[index];
                    count++;
                }
                if (foundUnknown) break;
            }
            if (foundUnknown) {
                downsizedGrid.data[y * downsizedGrid.info.width + x] = -1; // Mark the entire cell as -1 if any part is unknown
            } else if (count > 0) {
                downsizedGrid.data[y * downsizedGrid.info.width + x] = sum / count; // Average of the values
            } else {
                downsizedGrid.data[y * downsizedGrid.info.width + x] = -1; // Assign -1 if no valid data was found
            }
        }
    }

    return downsizedGrid;
}




// Callback functions to handle incoming grid data for each robot
void robot1GridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    

    std::lock_guard<std::mutex> lock(grid_mutex);
    if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0) {
        ROS_WARN("Received an invalid grid.");
        return;
    }
    geometry_msgs::Point start_position1;
    start_position1.x = 0.0;
    start_position1.y = 0.0;

     // Process the grid data for robot1
    latest_robot1_grid = *msg; // Store the latest grid


    // generate_path(start_position1, latest_robot1_grid, path_publisher1);
}

// Callback for Robot 1 starting position
void robot1StartPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(grid_mutex);
    start_position1 = *msg;
    ROS_INFO("Starting position for Robot 1 received - x: %f, y: %f", msg->x, msg->y);
}

void robot2GridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("Grid for Robot 2 received with width %d and height %d", msg->info.width, msg->info.height);
    std::lock_guard<std::mutex> lock(grid_mutex);
    if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0) {
        ROS_WARN("Received an invalid grid.");
        ROS_INFO("RRRRRRRRRRRRR2");
        return;
    }
    // geometry_msgs::Point start_position2;
    // start_position2.x = 19.0;
    // start_position2.y = 19.0;

    // Process the grid data for robot1
    latest_robot2_grid = *msg; // Store the latest grid
    


    generate_path(start_position2, latest_robot2_grid, path_publisher2);
}

// Callback for Robot 2 starting position
void robot2StartPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(grid_mutex);
    start_position2 = *msg;
    ROS_INFO("Starting position for Robot 2 received - x: %f, y: %f", msg->x, msg->y);
}

void robot3GridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(grid_mutex);
    if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0) {
        ROS_WARN("Received an invalid grid.");
        ROS_INFO("RRRRRRRRRRRRR3");
        return;
    }

    // Process the grid data for robot1
    latest_robot3_grid = *msg; // Store the latest grid


    generate_path(start_position3, latest_robot3_grid, path_publisher3);
}

// Callback for Robot 3 starting position
void robot3StartPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(grid_mutex);
    start_position3 = *msg;
    ROS_INFO("Starting position for Robot 3 received - x: %f, y: %f", msg->x, msg->y);
}

// Service callback
bool generate_path_service(coverage_path::GeneratePath::Request &req, coverage_path::GeneratePath::Response &res) {
    res.feedback = generate_path(req.point, latest_robot1_grid, path_publisher1);
    // res.feedback.message = "Path generation completed";
    return true;
}




/**
 * @brief Generate an optimal coverage path for a given area.
 * @param start The starting position of the path.
 * @param roi Pointer to a ROI to cover. Optional, if not given mission area is covered.
 * @return Whether the path has been generated successfully.
 */
bool generate_path (geometry_msgs::Point start, const nav_msgs::OccupancyGrid& robot_occupancy_map, ros::Publisher& path_publisher)
{
    NodeHandle nh;

    ROS_DEBUG("Starting at (%.2f,%.2f)", start.x, start.y);

    // get area divided per robot
    ROS_DEBUG("Get map of divided area...");

 
    nav_msgs::OccupancyGrid area = parseGrid(robot_occupancy_map,0.5);
    ROS_DEBUG("Grid has been downsized");
    // ROS_INFO("Occupancy Grid Info:");
    // ROS_INFO("  Width: %d", area.info.width);
    // ROS_INFO("  Height: %d", area.info.height);
    // ROS_INFO("  Resolution: %f meters/cell", area.info.resolution);
    // ROS_INFO("  Origin: (x: %f, y: %f, z: %f)", area.info.origin.position.x, area.info.origin.position.y, area.info.origin.position.z);

    ROS_INFO("Generate new coverage path...");

    // construct minimum spanning tree
    ROS_INFO("pre Construct minimum-spanning-tree...");
    spanning_tree tree;
    tree.initialize_graph(area, vertical);
    ROS_INFO("Construct minimum-spanning-tree...");
    tree.construct();
    ROS_INFO("postConstruct minimum-spanning-tree...");
    // visualize path
    if (visualize)
        mst_publisher.publish(tree.get_tree());

    // generate path
    ROS_INFO("Generate coverage path...");
    path.initialize_map(area, 0, vertical);
    // Check here
    path.initialize_tree(tree.get_mst_edges());
    if (!path.generate_path(start)) {
        ROS_WARN("Failed to generate path for the robot starting at (%.2f,%.2f)", start.x, start.y);
        return false;
        }
    if (turning_points)
        path.reduce();

    // visualize path
    // if (visualize)
    ROS_INFO("Visualize new coverage path...");

    path_publisher.publish(path.get_path());

    return true;
}

/**
 * @brief A ROS node that computes the optimal paths for area coverage with a swarm of CPSs.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char **argv)
{
    // init ros node
    init(argc, argv, "coverage_path");
    NodeHandle nh;

    path_publisher1 = nh.advertise<nav_msgs::Path>("coverage_path/path1", 1, true);
    // path_publisher2 = nh.advertise<nav_msgs::Path>("coverage_path/path2", 1, true);
    // path_publisher3 = nh.advertise<nav_msgs::Path>("coverage_path/path3", 1, true);
  

    // Subscribers for the divided maps
    ros::Subscriber robot1_sub = nh.subscribe("/robot1_grid", 10, robot1GridCallback);
    // ros::Subscriber robot2_sub = nh.subscribe("robot2_grid", 10, robot2GridCallback);
    // ros::Subscriber robot3_sub = nh.subscribe("robot3_grid", 10, robot3GridCallback);

    // Subscriber for the starting position of the Robots
    ros::Subscriber startPosSub_R1 = nh.subscribe("robot1_starting_pos", 10, robot1StartPositionCallback);
    // ros::Subscriber startPosSub_R2 = nh.subscribe("robot2_starting_pos", 10, robot2StartPositionCallback);
    // ros::Subscriber startPosSub_R3 = nh.subscribe("robot3_starting_pos", 10, robot3StartPositionCallback);

    // Create the service
    ros::ServiceServer service = nh.advertiseService("/generate_path", generate_path_service);



    ROS_INFO("/generate_path service server available");

    spin();

    return 0;
}
