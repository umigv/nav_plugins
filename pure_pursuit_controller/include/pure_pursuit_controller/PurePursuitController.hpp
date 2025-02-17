#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

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

        // member variables
        std::vector<xyCoord> path;
        std::vector<double> targetVelocities;
        xyCoord lastLookaheadPoint;
        double lastLookaheadPointIndex;
        bool pathFinished;

        // core functions
        void fillPath(std::vector<xyCoord> &path_in);
        void fillTargetVelocities();
        xyCoord getLookaheadPoint(xyCoord currentPt);
        std::vector<double> getLinearVelocity(xyCoord currentPt);
        std::vector<double> getAngularVelocity(xyCoord currentPt, double currentAngleRad, xyCoord lookaheadPt, std::vector<double> linearVelocity);

        // helper functions
        int getClosestPointIndex(xyCoord startingPt);
        double getArcCurvature(xyCoord currentPt, double currentAngleRad, xyCoord lookaheadPt);
        double getCurvatureAtPoint(xyCoord pt1, xyCoord pt2, xyCoord pt3);
        double getCurvatureAtPoint(int idx);
        int getSidePointIsOn(xyCoord currentPt, double currentAngleRad, xyCoord targetPt);
        void smoothPath(); // not priority

        // math functions
        int sgn(double num);
        int dot(std::vector<int> vec1, std::vector<int> vec2);
        double getAngleFromQuaternion(std::vector<double> q);
        double distanceBetweenPoints(int idx1, int idx2);
};
