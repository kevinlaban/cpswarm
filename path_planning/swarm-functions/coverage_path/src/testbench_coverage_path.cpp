#include <iostream>
#include "coverage_path.h"

int main() {
    // Create a test map
    std::vector<std::vector<int>> map = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };

    // Call the coverage_path function
    std::vector<std::pair<int, int>> path = coverage_path(map);

    // Print the resulting path
    std::cout << "Coverage Path: ";
    for (const auto& point : path) {
        std::cout << "(" << point.first << ", " << point.second << ") ";
    }
    std::cout << std::endl;

    return 0;
}

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
        // Calculate origin shift to center the first cell's midpoint under the new resolution
    double originShift = (roundedFactor * originalResolution) / 2;

    downsizedGrid.info.origin.position.x = originalGrid.info.origin.position.x + originShift - (originalResolution / 2);
    downsizedGrid.info.origin.position.y = originalGrid.info.origin.position.y + originShift - (originalResolution / 2);



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