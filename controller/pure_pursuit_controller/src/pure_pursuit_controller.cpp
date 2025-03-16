#include "pure_pursuit_controller.hpp"
#include <vector>

static auto toPose(const geometry_msgs::msg::Pose& pose) -> Pose {
    return Pose(pose.position.x, pose.position.y, pose.orientation.z);
}

static auto toDiscretePath(const std::vector<infra_common::CellCoordinate>& path) -> DiscretePath {
    std::vector<Point> convertedPath(path.size());
    std::transform(path.begin(), path.end(), convertedPath.begin(), [](const infra_common::CellCoordinate& coordinate) {
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

void PurePursuitController::set_path(const std::vector<infra_common::CellCoordinate>& path) {
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
