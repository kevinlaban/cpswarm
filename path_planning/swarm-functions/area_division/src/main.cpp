#include "lib/area_division.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include <string>
#include "area_division/DivideArea.h"

// Global variables to store the current map and a flag to check if the map is received
nav_msgs::OccupancyGrid current_map;
bool map_received = false;

// Callback function for receiving the map
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    current_map = *msg;
    map_received = true;
    ROS_INFO("Map received: width=%d, height=%d", current_map.info.width, current_map.info.height);
}

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

bool divideArea(area_division::DivideArea::Request &req, area_division::DivideArea::Response &res, tf::TransformListener &tf_listener) {
    if (!map_received) {
        res.success = false;
        res.message = "Map not yet received";
        return true;
    }

    std::vector<std::string> robot_frames = getRobotFrames(tf_listener);
    std::map<std::string, std::vector<int>> robot_positions;

    for (const auto& frame : robot_frames) {
        geometry_msgs::Point position;
        if (getRobotPosition(tf_listener, frame, position)) {
            robot_positions[frame] = {static_cast<int>(position.x), static_cast<int>(position.y)};
        } else {
            ROS_ERROR("Failed to get position for %s", frame.c_str());
            continue;
        }
    }

    // Initialize with current map and robot positions
    area_division ad;
    ad.initialize_map(current_map.info.width, current_map.info.height, current_map.data);
    ad.initialize_cps(robot_positions);
    ad.divide();

    // Process the divided areas and prepare response
    for (const auto& pos : robot_positions) {
        nav_msgs::OccupancyGrid divided_grid = ad.get_grid(current_map, pos.first);
        res.divided_maps.push_back(divided_grid);
    }

    res.success = true;
    res.message = "Area successfully divided";
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "area_division_service");
    ros::NodeHandle nh;

    tf::TransformListener tf_listener;

    ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);
    ros::ServiceServer service = nh.advertiseService<area_division::DivideArea::Request, area_division::DivideArea::Response>(
        "divide_area", boost::bind(divideArea, _1, _2, boost::ref(tf_listener))
    );

    ROS_INFO("Area division service ready.");

    ros::spin();

    return 0;
}
