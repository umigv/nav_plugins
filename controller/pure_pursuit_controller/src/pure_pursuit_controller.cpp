#include "pure_pursuit_controller.hpp"
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>

static auto yawFromQuartenion(const geometry_msgs::msg::Quaternion& quaternion_msg) -> double {
    tf2::Quaternion quaternion;
    quaternion.setValue(
        quaternion_msg.x,
        quaternion_msg.y,
        quaternion_msg.z,
        quaternion_msg.w 
    );
    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    return -1 * yaw;
}

static auto toPose(const geometry_msgs::msg::Pose& pose) -> Pose {
    const double yaw = yawFromQuartenion(pose.orientation);
    return Pose(pose.position.x, pose.position.y, yaw);
}

static auto toDiscretePath(const std::vector<geometry_msgs::msg::Point>& path) -> DiscretePath {
    std::vector<Point> convertedPath(path.size());
    std::transform(path.begin(), path.end(), convertedPath.begin(), [](const geometry_msgs::msg::Point& coordinate) {
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

void PurePursuitController::set_path(const std::vector<geometry_msgs::msg::Point>& path) {
    controller.setPath(toDiscretePath(path)); 
}

auto PurePursuitController::compute_next_command_velocity(
    const geometry_msgs::msg::Pose& current_pose, 
    [[maybe_unused]] const geometry_msgs::msg::Twist& current_velocity) -> geometry_msgs::msg::Twist {
    const Pose pose = toPose(current_pose);
    const Twist result = controller.step(pose);
    return toTwist(result);
}

auto PurePursuitController::is_finished() const -> bool {
    return controller.isFinished();
}

} // namespace controller_plugins
