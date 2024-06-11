#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <iomanip>
#include <algorithm> // Including <algorithm> for std::max

// Function prototypes
nav_msgs::OccupancyGrid padOccupancyGrid(const nav_msgs::OccupancyGrid& input_grid, int threshold, int iterations);
nav_msgs::OccupancyGrid reinforceObstacles(const nav_msgs::OccupancyGrid& originalGrid);
void printGrid(const nav_msgs::OccupancyGrid& grid);

// Function to pad the occupancy grid by expanding obstacles
nav_msgs::OccupancyGrid padOccupancyGrid(const nav_msgs::OccupancyGrid& input_grid, int threshold, int iterations) {
    nav_msgs::OccupancyGrid padded_grid = input_grid;
    int width = input_grid.info.width;
    int height = input_grid.info.height;

    for (int it = 0; it < iterations; ++it) {
        nav_msgs::OccupancyGrid temp_grid = padded_grid;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                if (padded_grid.data[index] > threshold) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            int new_x = x + dx;
                            int new_y = y + dy;
                            if (new_x >= 0 && new_x < width && new_y >= 0 && new_y < height) {
                                int new_index = new_y * width + new_x;
                                temp_grid.data[new_index] = 100;
                            }
                        }
                    }
                }
            }
        }
        padded_grid = temp_grid;
    }

    return padded_grid;
}

// Function to reinforce obstacles in the occupancy grid
nav_msgs::OccupancyGrid reinforceObstacles(const nav_msgs::OccupancyGrid& originalGrid) {
    nav_msgs::OccupancyGrid modifiedGrid = originalGrid; // Copy the original grid
    int width = originalGrid.info.width;
    int height = originalGrid.info.height;

    // Loop through each cell in the grid
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            int obstacleCount = 0;

            // Check all neighboring cells
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue; // Skip the current cell itself

                    int nx = x + dx;
                    int ny = y + dy;

                    // Make sure we are not out of bounds
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int neighborIndex = ny * width + nx;
                        if (originalGrid.data[neighborIndex] == 100) {
                            obstacleCount++;
                        }
                    }
                }
            }

            // If 5 or more neighbors are obstacles, set this cell to be an obstacle too
            if (obstacleCount >= 5) {
                modifiedGrid.data[index] = 100;
            }
        }
    }

    return modifiedGrid;
}

// Helper function to print the occupancy grid
void printGrid(const nav_msgs::OccupancyGrid& grid) {
    int width = grid.info.width;
    int height = grid.info.height;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            std::cout << std::setw(3) << static_cast<int>(grid.data[index]) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "test_pad_occupancy_grid");
    ros::NodeHandle nh;

    // Create a sample occupancy grid
    nav_msgs::OccupancyGrid input_grid;
    input_grid.info.width = 10;
    input_grid.info.height = 10;
    input_grid.data.resize(input_grid.info.width * input_grid.info.height, 0);

    // Set up a more complex test pattern
    // 0 0 0 0 0 0 0 0 0 0
    // 0 100 100 0 0 100 100 100 0 0
    // 0 100 0 0 0 100 0 0 100 0
    // 0 100 100 100 0 100 0 0 100 0
    // 0 0 0 0 0 100 0 0 0 0
    // 0 100 100 100 100 100 100 100 0 0
    // 0 100 0 0 0 0 0 100 100 0
    // 0 100 0 100 100 0 0 100 0 0
    // 0 100 100 100 0 0 100 0 0 0
    // 0 0 0 0 0 0 0 0 0 0

    std::vector<int> indices = {
        11, 12, 15, 16, 17, 
        21, 25, 28, 
        31, 32, 33, 35, 38, 
        51, 52, 53, 54, 55, 56, 57, 
        61, 68, 69, 
        71, 73, 74, 77, 
        81, 82, 83, 85, 
    };
    for (int idx : indices) {
        input_grid.data[idx] = 100;
    }

    // Print the grid before reinforcement
    std::cout << "Input Grid:" << std::endl;
    printGrid(input_grid);

    // Define threshold and iterations
    int threshold = 50;
    int iterations = 3;

    // Apply the padding function
    nav_msgs::OccupancyGrid padded_grid = padOccupancyGrid(input_grid, threshold, iterations);

    std::cout << "Padded Grid:" << std::endl;
    printGrid(padded_grid);

    // Apply the reinforceObstacles function
    nav_msgs::OccupancyGrid reinforced_grid = reinforceObstacles(input_grid);

    std::cout << "Reinforced Grid:" << std::endl;
    printGrid(reinforced_grid);

    return 0;
}
