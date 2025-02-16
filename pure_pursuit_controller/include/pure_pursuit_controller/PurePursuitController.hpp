#pragma once

#include <string>
#include <vector>
#include <memory>
#include <limits>

struct xyCoord {
    int x=0;
    int y=0;

    xyCoord operator-(const xyCoord& other) const {
        return {x - other.x, y - other.y};
    }
    xyCoord operator+(const xyCoord& other) const {
        return {x + other.x, y + other.y};
    }

    // allow conversion to int vector
    operator std::vector<int>() const {
        return {x, y};
    }
};

struct Twist {
    std::vector<double> linear = {0,0,0};
    std::vector<double> angular = {0,0,0};
};

struct Pose {
    std::vector<double> point = {0,0,0};
    std::vector<double> quaternion = {0,0,0,0};
};

struct Quaternion {
    double w = 0;
    double x = 0;
    double y = 0;
    double z = 0;
};

class PurePursuitController {
    public: 
        PurePursuitController();
        void setPath(std::vector<xyCoord> &path);
        Twist computeNextVelocityCmd(Pose pose, Twist velocity);
        bool isPathFinished();
        
    private:
        // Parameters:
        int spacing;
        double maxVelocity;
        double maxAcceleration;
        double trackWidth;
        int lookaheadDist;
        double kTurnConstant;

        std::vector<xyCoord> path;
        std::vector<double> targetVelocities;
        xyCoord lastLookaheadPoint;
        double lastLookaheadPointIndex;
        bool pathFinished;

        // priority
        void fillPath(std::vector<xyCoord> &path_in);
        void fillTargetVelocities();
        xyCoord getLookaheadPoint(xyCoord currentPt);
        std::vector<double> getLinearVelocity(xyCoord currentPt);
        std::vector<double> getAngularVelocity(xyCoord currentPt, double currentAngleRad, xyCoord lookaheadPt, std::vector<double> linearVelocity);

        // possible helper functions (or possibly not needed if I was doing too much earlier)
        void smoothPath(); // not priority
        int sgn(double num);
        int dot(std::vector<int> vec1, std::vector<int> vec2);
        double getAngleFromQuaternion(std::vector<double> q);
        int getSidePointIsOn(xyCoord currentPt, double currentAngleRad, xyCoord targetPt);
        // int getTurnError(int targetAngleDegrees, int currentHeadingDegrees);
        // int getLinearError(xyCoord targetPt, xyCoord currentPt);
        double distanceBetweenPoints(xyCoord pt1, xyCoord pt2);
        double distanceBetweenPoints(int idx1, int idx2);
        double getCurvatureAtPoint(xyCoord pt1, xyCoord pt2, xyCoord pt3);
        double getCurvatureAtPoint(int idx);

        double rateLimiter();
        xyCoord getLineIntersection(xyCoord pt1, xyCoord pt2);
        // Twist calculateTwistToPoint(xyCoord currentPt, int currentHeadingDegrees, xyCoord targetPt);
        int getClosestPointIndex(xyCoord startingPt);
        double getArcCurvature(xyCoord currentPt, double currentAngleRad, xyCoord lookaheadPt);
};
