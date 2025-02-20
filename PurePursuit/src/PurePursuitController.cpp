#include "PurePursuitController.hpp"
#include <vector>

static Pose toPose(const geometry_msgs::msg::Pose& pose){
    return Pose(pose.position.x, pose.position.y, pose.orientation.z);
}

static DiscretePath toDiscretePath(const std::vector<infra_common::CellCoordinate> &path){
    std::vector<Point> convertedPath(path.size());
    std::transform(path.begin(), path.end(), convertedPath.begin(), [](const infra_common::CellCoordinate& coordinate){
        return Point(coordinate.x, coordinate.y);
    });
    return DiscretePath(convertedPath);
}

static geometry_msgs::msg::Twist toTwist(const Twist& twist){
    geometry_msgs::msg::Twist result;
    result.linear.x = twist.linearVelocity;
    result.angular.z = twist.angularVelocity;
    return result;
}

PurePursuitController::PurePursuitController() : controller(PurePursuit::Gains(1, 1, 1, 1)){
    // TODO: Load gains from parameter server
}

void PurePursuitController::set_path(const std::vector<infra_common::CellCoordinate>& path){
    controller.setPath(toDiscretePath(path)); 
}

geometry_msgs::msg::Twist PurePursuitController::compute_next_command_velocity(
    const geometry_msgs::msg::Pose &current_pose, [[maybe_unused]] const geometry_msgs::msg::Twist& current_velocity){
    const Twist result = controller.step(toPose(current_pose));
    return toTwist(result);
}
