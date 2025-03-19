#include "pure_pursuit_controller.hpp"
#include <vector>
#include <cmath>

static auto toPose(const geometry_msgs::msg::Pose& pose) -> Pose {
    // https://stackoverflow.com/a/18115837
    const auto [x, y, z, w] = pose.orientation;
    const double yaw = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    
    return Pose(pose.position.x, pose.position.y, yaw);
}

static auto toDiscretePath(const std::vector<geometry_msgs::msg::Point> &path) -> DiscretePath {
    std::vector<Point> convertedPath(path.size());
    std::transform(path.begin(), path.end(), convertedPath.begin(), [](const geometry_msgs::msg::Point& coordinate){
        return Point(coordinate.x, coordinate.y);
    });
    return DiscretePath(convertedPath);
}

static auto toTwist(const Twist& twist) -> geometry_msgs::msg::Twist {
    geometry_msgs::msg::Twist result;
    result.linear.x = twist.linearVelocity;
    result.angular.z = twist.angularVelocity;
    return result;
}

namespace controller_plugins {

PurePursuitController::PurePursuitController() 
    : controller(PurePursuit::Gains(1, 1, 0.8, 0.152)) {
    // TODO: Load gains from parameter server
}

void PurePursuitController::set_path(const std::vector<geometry_msgs::msg::Point> &path){
    controller.setPath(toDiscretePath(path)); 
}

auto PurePursuitController::compute_next_command_velocity(
    const geometry_msgs::msg::Pose& current_pose, 
    [[maybe_unused]] const geometry_msgs::msg::Twist& current_velocity) -> geometry_msgs::msg::Twist {
    const Twist result = controller.step(toPose(current_pose));
    return toTwist(result);
}

auto PurePursuitController::is_finished() const -> bool {
    return controller.isFinished();
}

} // namespace controller_plugins
