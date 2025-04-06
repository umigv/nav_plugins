#pragma once
#include "plugin_base_classes/controller.hpp"
#include<optional>

namespace controller_plugins{

class EricPurePursuitController : public plugin_base_classes::Controller{
public:
    EricPurePursuitController();

    void set_path(const std::vector<geometry_msgs::msg::Point> &path) override;

    geometry_msgs::msg::Twist compute_next_command_velocity(
        const geometry_msgs::msg::Pose &current_pose, 
        [[maybe_unused]] const geometry_msgs::msg::Twist &current_velocity) override;

    bool is_finished() const override;

    private:
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q) const;

    std::optional<geometry_msgs::msg::Point> findLookaheadPoint(const geometry_msgs::msg::Pose &current_pose) const;
    
    double maxVelocity;
    double maxAngularVelocity;
    double lookaheadDistance;
    std::vector<geometry_msgs::msg::Point> path;
    bool isFinished = false;
};

} // namespace controller_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugins::EricPurePursuitController, plugin_base_classes::Controller)
