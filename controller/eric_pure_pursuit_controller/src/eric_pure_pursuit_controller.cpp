#include "eric_pure_pursuit_controller.hpp"
#include <cmath>

namespace controller_plugins{

EricPurePursuitController::EricPurePursuitController() : maxVelocity(0.4), maxAngularVelocity(0.4), lookaheadDistance(0.3){}

void EricPurePursuitController::set_path(const std::vector<geometry_msgs::msg::Point> &path){
    this->path = path;
    isFinished = false;
}

geometry_msgs::msg::Twist EricPurePursuitController::compute_next_command_velocity(
    const geometry_msgs::msg::Pose &current_pose, const geometry_msgs::msg::Twist& current_velocity){
    (void)current_velocity;  // Unused parameter
    auto lookahead_point = findLookaheadPoint(current_pose);

    if (!lookahead_point.has_value()) {
        isFinished = true;
        return geometry_msgs::msg::Twist();  // Stop
    }

    double local_x = lookahead_point->x - current_pose.position.x;
    double local_y = lookahead_point->y - current_pose.position.y;
    double curvature = 2 * local_y / (local_x * local_x + local_y * local_y);
    double dist = std::hypot(local_x, local_y);

    double linear = std::min(maxVelocity, dist);
    double angular = std::max(-maxAngularVelocity, std::min(maxAngularVelocity, linear * curvature));

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;

    return cmd;
}

bool EricPurePursuitController::is_finished() const{
    return isFinished;
}

double EricPurePursuitController::getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q) const {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::optional<geometry_msgs::msg::Point> EricPurePursuitController::findLookaheadPoint(const geometry_msgs::msg::Pose &current_pose) const{
    if (path.empty()) {
        return std::nullopt;
    }

    double x = current_pose.position.x;
    double y = current_pose.position.y;
    double yaw = getYawFromQuaternion(current_pose.orientation);

    for (const auto &point : path) {
        double gx = point.x;
        double gy = point.y;

        double dx = gx - x;
        double dy = gy - y;

        // Transform to robot's frame
        double local_x = std::cos(-yaw) * dx - std::sin(-yaw) * dy;
        double local_y = std::sin(-yaw) * dx + std::cos(-yaw) * dy;
        double dist = std::hypot(local_x, local_y);

        // Prevents driving backwards or directly to the side
        if (local_x > 0.05 && dist >= lookaheadDistance) {
            return point;
        }
    }

    return std::nullopt;
}

} // namespace controller_plugins