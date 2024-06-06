#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <iomanip>
#include <algorithm> // Including <algorithm> for std::max


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

    // Set some cells to a value above the threshold
    input_grid.data[12] = 70;
    input_grid.data[45] = 80;
    input_grid.data[78] = 90;

    std::cout << "Input Grid:" << std::endl;
    printGrid(input_grid);

    // Define threshold and iterations
    int threshold = 50;
    int iterations = 3;

    // Apply the padding function
    nav_msgs::OccupancyGrid padded_grid = padOccupancyGrid(input_grid, threshold, iterations);

    std::cout << "Padded Grid:" << std::endl;
    printGrid(padded_grid);

    return 0;
}
