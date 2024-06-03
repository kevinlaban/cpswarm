#include "lib/area_division.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include <string>
#include <mutex>
#include "your_package_name/PathPlanning.h"

std::mutex grid_mutex;
nav_msgs::OccupancyGrid current_map;
bool map_received = false;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(grid_mutex);
    current_map = *msg;
    map_received = true;
    ROS_INFO("Map received: width=%d, height=%d", current_map.info.width, current_map.info.height);
}

bool getRobotPosition(tf::TransformListener &listener, const std::string &robot_frame, geometry_msgs::Point &position) {
    tf::StampedTransform transform_base_to_odom, transform_odom_to_map;
    try {
        listener.waitForTransform("odom", robot_frame, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("odom", robot_frame, ros::Time(0), transform_base_to_odom);
        listener.waitForTransform("map", "odom", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("map", "odom", ros::Time(0), transform_odom_to_map);
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

bool divideAndPublish(your_package_name::PathPlanning::Request &req, your_package_name::PathPlanning::Response &res) {
    std::lock_guard<std::mutex> lock(grid_mutex);

    if (!map_received) {
        res.success = false;
        res.message = "No map received yet";
        return true;
    }

    tf::TransformListener tf_listener;
    area_division ad;
    std::vector<std::string> robot_frames = getRobotFrames(tf_listener);
    int num_robots_detected = robot_frames.size();

    if (num_robots_detected == 0) {
        res.success = false;
        res.message = "No robot frames found.";
        return true;
    }

    ad.initialize_map(current_map.info.width, current_map.info.height, current_map.data);
    std::map<std::string, std::vector<int>> updated_cps_positions;
    for (const auto& robot_frame : robot_frames) {
        std::string robot_name = robot_frame.substr(0, robot_frame.find('_'));
        geometry_msgs::Point position;
        if (getRobotPosition(tf_listener, robot_frame, position)) {
            updated_cps_positions[robot_name] = {static_cast<int>(position.x), static_cast<int>(position.y)};
        } else {
            ROS_ERROR("Failed to get position for %s", robot_name.c_str());
        }
    }
    ad.initialize_cps(updated_cps_positions);
    ad.divide();

    ros::NodeHandle nh;
    std::map<std::string, ros::Publisher> robot_pubs;
    std::map<std::string, ros::Publisher> start_pos_pubs;

    for (int i = 0; i < num_robots_detected; i++) {
        std::string robot_name = "robot" + std::to_string(i + 1);
        std::string grid_topic = "/" + robot_name + "_grid";
        std::string start_pos_topic = "/" + robot_name + "_starting_pos";
        ros::Publisher robot_pub = nh.advertise<nav_msgs::OccupancyGrid>(grid_topic, 10);
        ros::Publisher start_pos_pub = nh.advertise<geometry_msgs::Point>(start_pos_topic, 1);
        robot_pubs[robot_name] = robot_pub;
        start_pos_pubs[robot_name] = start_pos_pub;
    }

    for (const auto& robot_frame : robot_frames) {
        std::string robot_name = robot_frame.substr(0, robot_frame.find('_'));
        int i = 0;
        nav_msgs::OccupancyGrid robot_grid = ad.get_grid(current_map, robot_name);
        for (size_t j = 0; j < current_map.data.size(); ++j) {
            if (current_map.data[j] == -1) {
                robot_grid.data[j] = 100;
            }
        }
        std::string robot_name_publish = "robot" + std::to_string(i + 1);
        robot_pubs[robot_name_publish].publish(robot_grid);

        std::vector<int> coordinates = updated_cps_positions[robot_name];
        geometry_msgs::Point start_point;
        start_point.x = coordinates[0];
        start_point.y = coordinates[1];
        start_pos_pubs[robot_name_publish].publish(start_point);
        i += 1;
    }

    res.success = true;
    res.message = "Area divided and maps published successfully";
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "area_division_node");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);
    ros::ServiceServer service = nh.advertiseService("divide_area", divideAndPublish);

    ROS_INFO("Ready to divide area and publish maps.");

    ros::spin();

    return 0;
}
