#include "PurePursuitController.hpp"
#include <cmath>

// Public Functions ------------------------------------------------------------

PurePursuitController::PurePursuitController() {
    // TODO: parameters to tune
    spacing = 5;
    lookaheadDist = 10;
    kTurnConstant = 1;

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
    // also calculate target velocities at each point
    fillTargetVelocities();
}

Twist PurePursuitController::computeNextVelocityCmd(Pose pose, Twist velocity) {
    xyCoord currentPt = {pose.point[0], pose.point[1]};
    double currentDir = getAngleFromQuaternion(pose.quaternion);
    // get lookahead point as a target to get to
    xyCoord lookAheadPoint = getLookaheadPoint(currentPt);
    // calculate velocities to get to lookahead point
    Twist next_velocity;
    next_velocity.linear = getLinearVelocity(currentPt);
    next_velocity.angular = getAngularVelocity(currentPt, currentDir, lookAheadPoint, next_velocity.linear);
    // return Twist with velocities
    return next_velocity;
}

bool PurePursuitController::isPathFinished() {
    return pathFinished;
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

int PurePursuitController::dot(std::vector<int> vec1, std::vector<int> vec2) {
    if (vec1.size() != vec2.size()) {
        return 0;
    }
    int dotProduct = 0;
    for (int i = 0; i < vec1.size(); i++) {
        dotProduct += vec1[i] * vec2[i];
    }
    return dotProduct;
}

double PurePursuitController::getAngleFromQuaternion(std::vector<double> q) {
    // returns angle robot is facing in radians
    // assumes q is in the format (w, x, y, z) (typical format)
    return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

int PurePursuitController::getSidePointIsOn(xyCoord currentPt, double currentAngleRad, xyCoord targetPt) {
    // convention: positive means target point is on the left
    // side is found by sign of cross product of robot direction vector and robot to lookahead point vector
    xyCoord ptOnRobotLine = {currentPt.x + std::cos(currentAngleRad), currentPt.y + std::sin(currentAngleRad)};
    double crossProduct = (ptOnRobotLine.y - currentPt.y) * (targetPt.x - currentPt.x) - (ptOnRobotLine.x - currentPt.x) * (targetPt.y - currentPt.y);
    return -sgn(crossProduct); // TODO: check that this actually returns the correct side
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

double PurePursuitController::getCurvatureAtPoint(xyCoord pt1, xyCoord pt2, xyCoord pt3) {
    double x1 = pt1.x + 0.00001;
    double k1 = 0.5 * (x1 *x1 + pt1.y * pt1.y - pt2.x*pt2.x - pt2.y *pt2.y) / (x1 - pt2.x);
    double k2 = (pt1.y - pt2.y) / (x1 - pt2.x);
    double b = 0.5 * (pt2.x *pt2.x - 2 * pt2.x * k1 + pt2.y * pt2.y - pt3.x * pt3.x + 2 * pt3.x * k1 - pt3.y * pt3.y) / (pt3.x * k2 - pt3.y + pt2.y - pt2.x * k2);
    double a = k1 - k2 * b;
    double r = std::sqrt(std::pow(x1 - a, 2) + std::pow(pt1.y - b, 2));
    double c = 1/r;
    return c;
}

double PurePursuitController::getCurvatureAtPoint(int idx) {
    if (idx == 0 || idx == targetVelocities.size() - 1)
    {
        return 0;
    } else {
        return getCurvatureAtPoint(path.at(idx-1), path.at(idx), path.at(idx+1));
    }
    
}

double PurePursuitController::distanceBetweenPoints(int idx1, int idx2) {
    xyCoord pt1 = path.at(idx1), pt2 = path.at(idx2);
    return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
}

void PurePursuitController::fillTargetVelocities() {
    targetVelocities.resize(path.size());
    for (int i = 0; i < path.size(); ++i)
    {
        targetVelocities.at(i) = std::min(maxVelocity, kTurnConstant/getCurvatureAtPoint(i));
    } //fill original velocities based on curvature at point in path - step 1
    //step 2: rate limiter?
    
    //step 3: calculate new velocities
    double dist = 0;
    targetVelocities.at(targetVelocities.size() - 1) = 0;
    for (size_t i = targetVelocities.size() - 2; i < std::numeric_limits<size_t>::max(); --i) //maybe
    {
        dist = distanceBetweenPoints(i+1, i);
        targetVelocities.at(i) = std::min(targetVelocities.at(i), std::sqrt(std::pow(targetVelocities.at(i+1), 2) + 2 * maxAcceleration * dist));
    }
    
}

double PurePursuitController::rateLimiter() {
    
}

xyCoord PurePursuitController::getLineIntersection(xyCoord pt1, xyCoord pt2) {
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
    double sqrt_term = std::sqrt(lookaheadDist * lookaheadDist * dr2 - D * D);
    p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
    p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
    return p;
}

int PurePursuitController::getClosestPointIndex(xyCoord startingPt) {
    double minDist = std::numeric_limits<double>::infinity();
    int minDistIndex = 0;
    // could optimize by storing last closest point index and starting from there
    // did not optimize because I'm thinking about other things rn and just need this to work
    for (int i = 0; i < path.size(); i++) {
        int distance = sqrt(pow(path[i].x - startingPt.x, 2) + pow(path[i].y - startingPt.y, 2));
        if (distance < minDist) {
            minDist = distance;
            minDistIndex = i;
        }
    }
    return minDistIndex;
}

xyCoord PurePursuitController::getLookaheadPoint(xyCoord currentPt) {
    // find index of closest point, to start search at
    int closestPointIndex = getClosestPointIndex(currentPt);

    // loop through the next line segments looking for an intersection
    for (int i = closestPointIndex; i < path.size() - 1; i++) {
        // get line segment
        xyCoord segmentStartPt = path[i];
        xyCoord segmentEndPt = path[i+1];

        // calculate necessary vectors
        std::vector<int> segmentDir = segmentEndPt - segmentStartPt;
        std::vector<int> toStart = segmentStartPt - currentPt;

        // calculate discriminant
        int a = dot(segmentDir, segmentDir);
        int b = 2 * dot(toStart, segmentDir);
        int c = dot(toStart, toStart) - (lookaheadDist * lookaheadDist);
        int discriminant = b * b - 4 * a * c;

        // get t value of intersection
        double t = -1;
        if (discriminant >= 0) {
            discriminant = std::sqrt(discriminant);
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);

            if (t1 >= 0 && t1 <=1) {
                t = t1;
            }
            if (t2 >= 0 && t2 <=1) {
                t = t2;
            }
        }
        // if it's not a valid intersection, keep searching
        if (t < 0 || t > 1) {
            continue;
        }

        double fractionalIndex = i + t;
        if (fractionalIndex > lastLookaheadPointIndex) {
            xyCoord newLookAheadPoint = segmentStartPt + xyCoord{int(t * segmentDir[0]), int(t * segmentDir[1])};
            lastLookaheadPoint = newLookAheadPoint;
            lastLookaheadPointIndex = fractionalIndex;
            return newLookAheadPoint;
        }
    }

    // if no lookahead point has been returned, return the last lookahead point
    return lastLookaheadPoint;
}

double PurePursuitController::getArcCurvature(xyCoord currentPt, double currentAngleRad, xyCoord lookaheadPt) {
    /* 
    imagine a triangle with sides: horizontal, vertical, hypotenuse, 
    with the robot at the corner of horizontal and hypotenuse and facing in the vertical direction,
    and the lookahead point at the corner of vertical and hypotenuse.
    hypotenuse is lookAheadDist.
    horizontal is horizontalOffset.
    */

    // calculate horizontal offset from lookahead point
    double a = -std::tan(currentAngleRad);
    double b = 1;
    double c = std::tan(currentAngleRad) * currentPt.x - currentPt.y;
    double horizontalOffset = std::abs(a * lookaheadPt.x + b * lookaheadPt.y + c) / std::sqrt(a * a + b * b);

    // calculate and return curvature
    return (2 * horizontalOffset) / (lookaheadDist * lookaheadDist);
}

std::vector<double> PurePursuitController::getLinearVelocity(xyCoord currentPt) {
    // returned vector should be of length 3 for ROS (velocity along x,y,z axes)
    int idx = getClosestPointIndex(currentPt);
    // TODO: check axis of travel is as assumed (if not, change this assumption in getAngularVelocity() as well)
    return {targetVelocities.at(idx), 0, 0}; //maybe
}

std::vector<double> PurePursuitController::getAngularVelocity(xyCoord currentPt, double currentAngleRad, xyCoord lookaheadPt, std::vector<double> linearVelocity) {
    // returned vector should be of length 3 for ROS (angular velocity around x,y,z axes)
    // assuming positive angular.z turns the robot left (checked with embedded, positive is counter clockwise like the unit circle)

    double curvature = getArcCurvature(currentPt, currentAngleRad, lookaheadPt);
    int side = getSidePointIsOn(currentPt, currentAngleRad, lookaheadPt);
    double signedCurvature = side * curvature;

    double angularVelocity = curvature * linearVelocity[0];
    return {0, 0, angularVelocity};
}
