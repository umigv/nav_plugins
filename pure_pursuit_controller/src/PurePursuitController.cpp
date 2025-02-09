#include "PurePursuitController.hpp"
#include <cmath>

int PurePursuitController::sgn(double num) {
    if (num < 0)
    {
        return -1;
    } else {
        return 1;
    }
    
}

int PurePursuitController::getTurnError(int targetAngleDegrees, int currentHeadingDegrees) {
    int turnAngle = targetAngleDegrees - currentHeadingDegrees;
    if (turnAngle < -180 || turnAngle > 180) {
        turnAngle = -1 * sgn(turnAngle) * (360 - std::abs(turnAngle));
    }
    return turnAngle;
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

PurePursuitController::PurePursuitController(std::vector<xyCoord> &path) {

}