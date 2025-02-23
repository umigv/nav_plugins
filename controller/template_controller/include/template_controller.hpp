#pragma once
#include "plugin_base_classes/controller.hpp"

namespace controller_plugins{

class TemplateController : public plugin_base_classes::Controller{
public:
    void set_path(const std::vector<infra_common::CellCoordinate> &path) override;

    geometry_msgs::msg::Twist compute_next_command_velocity(
        const geometry_msgs::msg::Pose &current_pose, 
        [[maybe_unused]] const geometry_msgs::msg::Twist &current_velocity) override;

    bool is_finished() const override;
};

} // namespace controller_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugins::TemplateController, plugin_base_classes::Controller)
