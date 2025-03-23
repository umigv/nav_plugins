#pragma once
#include "plugin_base_classes/controller.hpp"

#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace controller_plugins{

class PurePursuitController : public plugin_base_classes::Controller{
public:
    void set_path(const std::vector<geometry_msgs::msg::Point> &path) override;

    geometry_msgs::msg::Twist compute_next_command_velocity(
        const geometry_msgs::msg::Pose &current_pose, 
        [[maybe_unused]] const geometry_msgs::msg::Twist &current_velocity) override;

    bool is_finished() const override;

private:
    // Parameters:
    double spacing;
    double maxVelocity;
    double maxAcceleration;
    double trackWidth;
    double lookaheadDist;
    double kTurnConstant;

    // member variables
    std::vector<geometry_msgs::msg::Point> path;
    std::vector<double> targetVelocities;
    geometry_msgs::msg::Point lastLookaheadPoint;
    double lastLookaheadPointIndex;
    bool pathFinished;

    // core functions
    void fillPath(const std::vector<geometry_msgs::msg::Point> &path_in);
    void fillTargetVelocities();
    geometry_msgs::msg::Point getLookaheadPoint(geometry_msgs::msg::Point currentPt);
    geometry_msgs::msg::Vector3 getLinearVelocity(geometry_msgs::msg::Point currentPt);
    geometry_msgs::msg::Vector3 getAngularVelocity(geometry_msgs::msg::Point currentPt, double currentAngleRad, geometry_msgs::msg::Point lookaheadPt, geometry_msgs::msg::Vector3 linearVelocity);

    // helper functions
    size_t getClosestPointIndex(geometry_msgs::msg::Point startingPt);
    double getArcCurvature(geometry_msgs::msg::Point currentPt, double currentAngleRad, geometry_msgs::msg::Point lookaheadPt);
    double getCurvatureAtPoint(geometry_msgs::msg::Point pt1, geometry_msgs::msg::Point pt2, geometry_msgs::msg::Point pt3);
    double getCurvatureAtPoint(size_t idx);
    int getSidePointIsOn(geometry_msgs::msg::Point currentPt, double currentAngleRad, geometry_msgs::msg::Point targetPt);
    void smoothPath(); // not priority

    // math functions
    int sgn(double num);
    int dot(std::vector<double> vec1, std::vector<double> vec2);
    double getAngleFromQuaternion(geometry_msgs::msg::Quaternion q);
    double distanceBetweenPoints(int idx1, int idx2);
};

} // namespace controller_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugins::PurePursuitController, plugin_base_classes::Controller)
