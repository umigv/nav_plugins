#pragma once

#include <string>
#include <vector>
#include <memory>

struct xyCoord {
    int x=0;
    int y=0;
};

struct Twist {
    double linear[3] = {0};
    double angular[3] = {0};
};

struct Pose {
    double point[3] = {0};
    double quaternion[4] = {0};
};

class PurePursuitController {
    public: 
        PurePursuitController();
        void setPath(std::vector<xyCoord> &path);
        Twist computeNextVelocityCmd(Pose pose, Twist velocity);
        
    private:
        // Parameters:
        int spacing;
        double lookAheadDist;
        float maxVelocity;
        float maxAcceleration;
        float trackWidth;

        std::vector<xyCoord> path;
        std::vector<double> targetVelocities;
        xyCoord lastLookaheadPoint;
        int lastLookaheadPointIndex;
        bool pathFinished;

        void fillPath(std::vector<xyCoord> &path_in);
        void smoothPath(); // not priority

        int sgn(double num);
        // int getTurnError(int targetAngleDegrees, int currentHeadingDegrees);
        // int getLinearError(xyCoord targetPt, xyCoord currentPt);
        double distanceBetweenPoints(xyCoord pt1, xyCoord pt2);
        double getCurvatureAtPoint(xyCoord pt);
        void fillTargetVelocities();
        double rateLimiter();

        xyCoord getLineIntersection(xyCoord pt1, xyCoord pt2, double lookAheadDist);
        // Twist calculateTwistToPoint(xyCoord currentPt, int currentHeadingDegrees, xyCoord targetPt);
        xyCoord getClosestPoint(xyCoord startingPt);
        xyCoord getLookaheadPoint();
        double getArcCurvature();
        double* getLinearVelocity();
        double* getAngularVelocity();
};