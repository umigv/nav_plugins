#include "template_controller.hpp"

namespace controller_plugins{

void TemplateController::set_path(const std::vector<geometry_msgs::msg::Point> &path){
    (void)(path);
}

geometry_msgs::msg::Twist TemplateController::compute_next_command_velocity(
    const geometry_msgs::msg::Pose &current_pose, const geometry_msgs::msg::Twist& current_velocity){
    (void)(current_pose);
    (void)(current_velocity);
    geometry_msgs::msg::Twist command_velocity;
    return command_velocity;
}

bool TemplateController::is_finished() const{
    return true;
}

} // namespace controller_plugins