#pragma once
#include "plugin_base_classes/controller.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "PurePursuit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

class PurePursuitController final : public rclcpp::Node {
public:
    PurePursuitController();

    void set_path(const std::vector<geometry_msgs::msg::Point>& path);

    void step();

    auto is_finished() const -> bool;
    
protected:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    
    PurePursuit controller;
    geometry_msgs::msg::Pose current_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};
