#pragma once
#include "plugin_base_classes/controller.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace controller_plugins{

class PurePursuitController : public plugin_base_classes::Controller{
public:
    void set_path(const std::vector<infra_common::CellCoordinate> &path) override;

    geometry_msgs::msg::Twist compute_next_command_velocity(
        const geometry_msgs::msg::Pose &current_pose, 
        [[maybe_unused]] const geometry_msgs::msg::Twist &current_velocity) override;

    bool is_finished() const override;

private:
    // Parameters:
    int spacing;
    double maxVelocity;
    double maxAcceleration;
    double trackWidth;
    int lookaheadDist;
    double kTurnConstant;

    // member variables
    std::vector<infra_common::CellCoordinate> path;
    std::vector<double> targetVelocities;
    infra_common::CellCoordinate lastLookaheadPoint;
    double lastLookaheadPointIndex;
    bool pathFinished;

    // core functions
    void fillPath(const std::vector<infra_common::CellCoordinate> &path_in);
    void fillTargetVelocities();
    infra_common::CellCoordinate getLookaheadPoint(infra_common::CellCoordinate currentPt);
    geometry_msgs::msg::Vector3 getLinearVelocity(infra_common::CellCoordinate currentPt);
    geometry_msgs::msg::Vector3 getAngularVelocity(infra_common::CellCoordinate currentPt, double currentAngleRad, infra_common::CellCoordinate lookaheadPt, geometry_msgs::msg::Vector3 linearVelocity);

    // helper functions
    size_t getClosestPointIndex(infra_common::CellCoordinate startingPt);
    double getArcCurvature(infra_common::CellCoordinate currentPt, double currentAngleRad, infra_common::CellCoordinate lookaheadPt);
    double getCurvatureAtPoint(infra_common::CellCoordinate pt1, infra_common::CellCoordinate pt2, infra_common::CellCoordinate pt3);
    double getCurvatureAtPoint(size_t idx);
    int getSidePointIsOn(infra_common::CellCoordinate currentPt, double currentAngleRad, infra_common::CellCoordinate targetPt);
    void smoothPath(); // not priority

    // math functions
    int sgn(double num);
    int dot(std::vector<int> vec1, std::vector<int> vec2);
    double getAngleFromQuaternion(geometry_msgs::msg::Quaternion q);
    double distanceBetweenPoints(int idx1, int idx2);
};

} // namespace controller_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugins::PurePursuitController, plugin_base_classes::Controller)
