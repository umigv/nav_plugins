#pragma once
#include "plugin_base_classes/controller.hpp"
#include "PurePursuit.hpp"

class PurePursuitController final : public plugin_base_classes::Controller{
public:
    PurePursuitController();

    void set_path(const std::vector<infra_common::CellCoordinate> &path) override;

    geometry_msgs::msg::Twist compute_next_command_velocity(
        const geometry_msgs::msg::Pose &current_pose, const geometry_msgs::msg::Twist &current_velocity) override;

protected:
    PurePursuit controller;
};