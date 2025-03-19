#pragma once
#include "plugin_base_classes/controller.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "PurePursuit.hpp"

namespace controller_plugins {

class PurePursuitController final : public plugin_base_classes::Controller {
public:
    PurePursuitController();

    void set_path(const std::vector<geometry_msgs::msg::Point>& path) override;

    auto compute_next_command_velocity(const geometry_msgs::msg::Pose& current_pose, 
                                       [[maybe_unused]] const geometry_msgs::msg::Twist& current_velocity) 
        -> geometry_msgs::msg::Twist override;

    auto is_finished() const -> bool override;
    
protected:
    PurePursuit controller;
};

} // namespace controller_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugins::PurePursuitController, plugin_base_classes::Controller)
