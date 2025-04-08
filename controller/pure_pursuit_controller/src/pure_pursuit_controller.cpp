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

PurePursuitController::PurePursuitController() 
    : rclcpp::Node("Pure_Pursuit_Controller"), controller(PurePursuit::Gains(0.4, 100, 2, 0.3)) {

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&PurePursuitController::path_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PurePursuitController::step, this)
    );
}

void PurePursuitController::set_path(const std::vector<geometry_msgs::msg::Point>& path) {
    controller.setPath(toDiscretePath(path)); 
}

void PurePursuitController::step() {
    const Pose pose = toPose(current_pose);
    const Twist result = controller.step(pose);
    cmd_pub_->publish(toTwist(result));
}

auto PurePursuitController::is_finished() const -> bool {
    return controller.isFinished();
}

void PurePursuitController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    current_pose = msg->pose.pose;
}

void PurePursuitController::path_callback(const nav_msgs::msg::Path::SharedPtr msg){
    std::vector<geometry_msgs::msg::Point> path;
    for (const auto& pose : msg->poses) {
        path.push_back(pose.pose.position);
    }

    set_path(path);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}