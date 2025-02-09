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

class PurePursuitController {
    public: 
        PurePursuitController(std::vector<xyCoord> &path);
    
    private:
        // helper functions
        int sgn(double num);
        int getTurnError(int targetAngleDegrees, int currentHeadingDegrees);
        int getLinearError(xyCoord targetPt, xyCoord currentPt);

        xyCoord getLineIntersection(xyCoord pt1, xyCoord pt2, double lookAheadDist);
        Twist calculateTwistToPoint(xyCoord currentPt, int currentHeadingDegrees, xyCoord targetPt);
};