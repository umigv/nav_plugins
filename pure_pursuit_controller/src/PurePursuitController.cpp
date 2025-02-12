#include "PurePursuitController.hpp"
#include <cmath>

// Public Functions ------------------------------------------------------------

PurePursuitController::PurePursuitController() {
    // TODO: parameters to tune
    spacing = 5;

    // high priority TODO: parameters to fill in
    maxVelocity = 2;
    maxAcceleration = 2;
    trackWidth = 2;

    // general initialization
    pathFinished = false;
}

void PurePursuitController::setPath(std::vector<xyCoord> &path) {
    fillPath(path);
    lastLookaheadPoint = path[0];
    lastLookaheadPointIndex = 0;
}

Twist PurePursuitController::computeNextVelocityCmd(Pose pose, Twist velocity) {
    
}


// Private Functions -----------------------------------------------------------

void PurePursuitController::fillPath(std::vector<xyCoord> &path_in) {
    for (size_t i = 0; i < path_in.size() - 1; i++)
    {
        xyCoord start = path_in[i], next = path_in[i+1];
        xyCoord vec{next.x - start.x, next.y - start.y};
        int magnitude = sqrt(vec.x * vec.x + vec.y * vec.y);
        int numPts = ceil(magnitude / spacing);
        vec.x = vec.x / magnitude * spacing;
        vec.y = vec.y / magnitude * spacing;
        for (size_t i = 0; i < numPts; ++i)
        {
            vec.x = start.x + vec.x * i;
            vec.y = start.y + vec.y * i;
            path.push_back(vec);
        }
    }
    path.push_back(path_in[path_in.size() - 1]);
}

void PurePursuitController::smoothPath() {
    // not a priority
}

int PurePursuitController::sgn(double num) {
    if (num < 0)
    {
        return -1;
    } else {
        return 1;
    }
    
}

// int PurePursuitController::getTurnError(int targetAngleDegrees, int currentHeadingDegrees) {
//     int turnAngle = targetAngleDegrees - currentHeadingDegrees;
//     if (turnAngle < -180 || turnAngle > 180) {
//         turnAngle = -1 * sgn(turnAngle) * (360 - std::abs(turnAngle));
//     }
//     return turnAngle;
// }

// int PurePursuitController::getLinearError(xyCoord targetPt, xyCoord currentPt) {

// }

double PurePursuitController::distanceBetweenPoints(xyCoord pt1, xyCoord pt2) {
    return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
}

double PurePursuitController::getCurvatureAtPoint(xyCoord pt) {
    
}

void PurePursuitController::fillTargetVelocities() {

}

double PurePursuitController::rateLimiter() {
    
}

xyCoord PurePursuitController::getLineIntersection(xyCoord pt1, xyCoord pt2, double lookAheadDist) {
    // // subtract currentpos from points to offset system to origin
    // xyCoord pt1{pt1o.x - currentPos.x, pt1o.y - currentPos.y}, pt2{pt2o.x - currentPos.x, pt2o.y - currentPos.y};
    // double dY = pt2.y - pt1.y, dX = pt2.x - pt1.x, solx1, solx2, soly1, soly2;
    // double lineDist = sqrt(((pt2.x - pt1.x) * (pt2.x - pt1.x)) + ((pt2.y - pt1.y) * (pt2.y - pt1.y)));
    // double det = pt1.x * pt2.y - pt2.x * pt1.y;
    // double discriminant = pow(lookAheadDist, 2) * pow(lineDist, 2) - pow(det, 2);
    // double dr = sqrt(dX*dX + dY*dY);
    // xyCoord sol1, sol2;
    // if (discriminant > 0 ) //possible intersection
    // {
    //     /* find x,y of intersection */
    //     solx1 = (det * dY + sgn(dY) * dX * sqrt(discriminant))/(dr*dr);
    //     solx2 = (det * dY - sgn(dY) * dX * sqrt(discriminant))/(dr*dr);
    //     soly1 = (-det * dX + abs(dY) * sqrt(discriminant))/(dr*dr);
    //     soly2 = (-det * dX - abs(dY) * sqrt(discriminant))/(dr*dr);
    //     sol1 = {solx1 + currentPos.x, soly1 + currentPos.y};
    //     sol2 = {solx2 + currentPos.x, soly2 + currentPos.y};
    //     // check if x,y are within bounds
        
    // }
    double x1 = pt1.x;
    double x2 = pt2.x;
    double y1 = pt1.y;
    double y2 = pt2.y;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr2 = dx * dx + dy * dy;
    double D = x1 * y2 - x2 * y1;

    // Augmentation to only return point within segment
    double d1 = x1 * x1 + y1 * y1;
    double d2 = x2 * x2 + y2 * y2;
    double dd = d2 - d1;

    xyCoord p;
    double sqrt_term = std::sqrt(lookAheadDist * lookAheadDist * dr2 - D * D);
    p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
    p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
    return p;
}

xyCoord PurePursuitController::getClosestPoint(xyCoord startingPt) {
    
}

xyCoord PurePursuitController::getLookaheadPoint() {
    
}

double PurePursuitController::getArcCurvature() {
    
}

double* PurePursuitController::getLinearVelocity() {
    
}

double* PurePursuitController::getAngularVelocity() {
    
}
