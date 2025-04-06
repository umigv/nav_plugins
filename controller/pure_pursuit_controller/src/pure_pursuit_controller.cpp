#include "pure_pursuit_controller.hpp"

namespace controller_plugins{

void PurePursuitController::set_path(const std::vector<geometry_msgs::msg::Point>& path){
    RCLCPP_INFO(rclcpp::get_logger("PurePursuitController"), "PurePursuitController setting path");

    // TODO: parameters to tune
    spacing = 0.01;
    lookaheadDist = .3;
    kTurnConstant = 3;

    // high priority TODO: parameters to fill in
    maxVelocity = 1;
    maxAcceleration = 1;

    // general initialization
    pathFinished = false;
    lastLookaheadPoint = path[0];
    lastLookaheadPointIndex = 0;

    // fill path variable
    fillPath(path);

    // also calculate target velocities at each point
    fillTargetVelocities();
}

geometry_msgs::msg::Twist PurePursuitController::compute_next_command_velocity(
    const geometry_msgs::msg::Pose &current_pose, const geometry_msgs::msg::Twist& current_velocity){
    RCLCPP_INFO(rclcpp::get_logger("PurePursuitController"), "PurePursuitController computing next velocity");
    (void)(current_velocity); // current velocity not used

    // get info from parameters
    geometry_msgs::msg::Point currentPt = current_pose.position;
    // double currentDir = getAngleFromQuaternion(current_pose.orientation);
    tf2::Quaternion q;
    tf2::fromMsg(current_pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double currentDir = yaw;

    // check if this is the last step of the path
    if (getClosestPointIndex(currentPt) == path.size() - 1) {
        pathFinished = true;
    }

    // get lookahead point as a target to get to
    geometry_msgs::msg::Point lookAheadPoint = getLookaheadPoint(currentPt);

    // calculate velocities to get to lookahead point
    geometry_msgs::msg::Twist next_velocity;
    next_velocity.linear = getLinearVelocity(currentPt);
    next_velocity.angular = getAngularVelocity(currentPt, currentDir, lookAheadPoint, next_velocity.linear);

    // return Twist with velocities
    return next_velocity;
}

bool PurePursuitController::is_finished() const{
    return pathFinished;
}

// Core Functions --------------------------------------------------------------

void PurePursuitController::fillPath(const std::vector<geometry_msgs::msg::Point> &path_in) {
    for (size_t i = 0; i < path_in.size() - 2; i++)
    {
        // std::cout << "Path " << i << " coordinates: " << path_in[i].x << ", " << path_in[i].y << std::endl;
        geometry_msgs::msg::Point start = path_in[i], next = path_in[i+1];
        geometry_msgs::msg::Point vec, pathPoint;
        vec.x = next.x - start.x;
        vec.y = next.y - start.y;
        vec.z = 0.0;
        double magnitude = sqrt(vec.x * vec.x + vec.y * vec.y);
        int numPts = ceil(magnitude / spacing);
        vec.x = vec.x / magnitude * spacing;
        vec.y = vec.y / magnitude * spacing;
        // std::cout << "Path " << i << " coordinates: " << vec.x << ", " << vec.y << std::endl
        //  << "NumPoints: " << numPts << std::endl;
        for (int j = 0; j < numPts; ++j)
        {
            // std::cout << "start at path_in[" << i << "] coordinates: " << start.x << ", " << start.y << std::endl;
            pathPoint.x = start.x + vec.x * j;
            pathPoint.y = start.y + vec.y * j;
            // std::cout << "VelPath " << i+j << " coordinates: " << pathPoint.x << ", " << pathPoint.y << std::endl;
            path.push_back(pathPoint);
        }
    }
    path.push_back(path_in[path_in.size() - 1]);
}

void PurePursuitController::fillTargetVelocities() {
    targetVelocities.resize(path.size());
    for (size_t i = 0; i < path.size(); ++i)
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

//TODO: getLookaheadPoint() not working rn
geometry_msgs::msg::Point PurePursuitController::getLookaheadPoint(geometry_msgs::msg::Point currentPt) {
    // find index of closest point, to start search at
    int closestPointIndex = getClosestPointIndex(currentPt);
    // loop through the next line segments looking for an intersection
    for (size_t i = closestPointIndex; i < path.size() - 1; i++) {
        // get line segment
        geometry_msgs::msg::Point segmentStartPt = path[i];
        geometry_msgs::msg::Point segmentEndPt = path[i+1];
        
        // calculate necessary vectors
        std::vector<double> segmentDir = {segmentEndPt.x - segmentStartPt.x, segmentEndPt.y - segmentStartPt.y};
        std::vector<double> toStart = {segmentStartPt.x - currentPt.x, segmentStartPt.y - currentPt.y};

        // calculate discriminant
        double a = dot(segmentDir, segmentDir);
        double b = 2 * dot(toStart, segmentDir);
        double c = dot(toStart, toStart) - (lookaheadDist * lookaheadDist);
        double discriminant = b * b - 4 * a * c;

        // get t value of intersection
        double t = -1;
        if (discriminant >= 0) { // check these maybe???
            discriminant = std::sqrt(discriminant);
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a); 

            if (t1 >= 0 && t1 <=1) {
                t = t1;
                std::cout << "POTENTIAL INTERSECTION: " << t1 << "\n";
            }
            if (t2 >= 0 && t2 <=1) {
                t = t2;
                std::cout << "POTENTIAL INTERSECTION: " << t2 << "\n";
            } // these never run either
        }
        // if it's not a valid intersection, keep searching
        if (t < 0 || t > 1) {
            continue;
        } else {
            std::cout << "VALID INTERSECTION\n"; //never called maybe?
        }
        double fractionalIndex = i + t;
        std::cout << "ClosestPtIDX: " << closestPointIndex << "\nFractionalIDX: " << fractionalIndex << std::endl;
        if (fractionalIndex >= lastLookaheadPointIndex) {
            geometry_msgs::msg::Point p;
            p.x = t * segmentDir[0];
            p.y = t * segmentDir[1];
            p.z = 0.0;
            geometry_msgs::msg::Point newLookAheadPoint;
            newLookAheadPoint.x = segmentStartPt.x + p.x;
            newLookAheadPoint.y = segmentStartPt.y + p.y;
            newLookAheadPoint.z = 0.0;
            lastLookaheadPoint = newLookAheadPoint;
            lastLookaheadPointIndex = fractionalIndex;
            std::cout << "New Lookahead Point: " << newLookAheadPoint.x << ", " << newLookAheadPoint.y << std::endl;
            return newLookAheadPoint; 
        }
    }
    std::cout << "Last Lookahead Point: " << lastLookaheadPoint.x << ", " << lastLookaheadPoint.y << std::endl;
    // if no lookahead point has been returned, return the last lookahead point
    return lastLookaheadPoint; // maybe return optional rather than last??? idk
}

geometry_msgs::msg::Vector3 PurePursuitController::getLinearVelocity(geometry_msgs::msg::Point currentPt) {
    // returned vector should be of length 3 for ROS (velocity along x,y,z axes)
    int idx = getClosestPointIndex(currentPt);
    // TODO: check axis of travel is as assumed (if not, change this assumption in getAngularVelocity() as well)
    geometry_msgs::msg::Vector3 v;
    v.x = targetVelocities.at(idx);
    v.y = 0;
    v.z = 0;
    return v;
}

geometry_msgs::msg::Vector3 PurePursuitController::getAngularVelocity(
    geometry_msgs::msg::Point currentPt, double currentAngleRad, 
    geometry_msgs::msg::Point lookaheadPt, geometry_msgs::msg::Vector3 linearVelocity) {
    // returned vector should be of length 3 for ROS (angular velocity around x,y,z axes)
    // assuming positive angular.z turns the robot left (checked with embedded, positive is counter clockwise like the unit circle)

    double curvature = getArcCurvature(currentPt, currentAngleRad, lookaheadPt); // returns 0 due to not recieving lookahead pt
    int side = getSidePointIsOn(currentPt, currentAngleRad, lookaheadPt);
    double signedCurvature = side * curvature;
    double angularVelocity = signedCurvature * linearVelocity.x;
    std::cout << "lookahead point: " << lookaheadPt.x << ", " << lookaheadPt.y << "\nCurrentAngleRad: " << currentAngleRad << "\nsignedCurvature: " << signedCurvature <<"\nAngular Velocity: " << angularVelocity << std::endl;
    geometry_msgs::msg::Vector3 v;
    v.x = 0;
    v.y = 0;
    v.z = std::min(angularVelocity, 1.0);
    return v;
}

// Helper Functions ------------------------------------------------------------

size_t PurePursuitController::getClosestPointIndex(geometry_msgs::msg::Point startingPt) {
    double minDist = std::numeric_limits<double>::infinity();
    size_t minDistIndex = 0;
    // could optimize by storing last closest point index and starting from there
    // did not optimize because I'm thinking about other things rn and just need this to work
    for (size_t i = 0; i < path.size(); i++) {
        double distance = sqrt(pow(path[i].x - startingPt.x, 2) + pow(path[i].y - startingPt.y, 2));
        // std::cout << "Distance from " << startingPt.x << ", " << startingPt.y << " to " << path[i].x << ", " << path[i].y << ": " << distance << std::endl;
        if (distance < minDist) {
            minDist = distance;
            minDistIndex = i;
        }
    }
    return minDistIndex;
}

double PurePursuitController::getArcCurvature(geometry_msgs::msg::Point currentPt, double currentAngleRad, geometry_msgs::msg::Point lookaheadPt) {
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

double PurePursuitController::getCurvatureAtPoint(geometry_msgs::msg::Point pt1, geometry_msgs::msg::Point pt2, geometry_msgs::msg::Point pt3) {
    double x1 = pt1.x + 0.00001;
    double k1 = 0.5 * (x1 *x1 + pt1.y * pt1.y - pt2.x*pt2.x - pt2.y *pt2.y) / (x1 - pt2.x);
    double k2 = (pt1.y - pt2.y) / (x1 - pt2.x);
    double b = 0.5 * (pt2.x *pt2.x - 2 * pt2.x * k1 + pt2.y * pt2.y - pt3.x * pt3.x + 2 * pt3.x * k1 - pt3.y * pt3.y) / (pt3.x * k2 - pt3.y + pt2.y - pt2.x * k2);
    double a = k1 - k2 * b;
    double r = std::sqrt(std::pow(x1 - a, 2) + std::pow(pt1.y - b, 2));
    double c = 1/r;
    return c;
}

double PurePursuitController::getCurvatureAtPoint(size_t idx) {
    if (idx == 0 || idx == targetVelocities.size() - 1) {
        return 0;
    } else {
        return getCurvatureAtPoint(path.at(idx-1), path.at(idx), path.at(idx+1));
    }
}

int PurePursuitController::getSidePointIsOn(geometry_msgs::msg::Point currentPt, double currentAngleRad, geometry_msgs::msg::Point targetPt) {
    // convention: positive means target point is on the left
    // side is found by sign of cross product of robot direction vector and robot to lookahead point vector
    geometry_msgs::msg::Point ptOnRobotLine;
    ptOnRobotLine.x = currentPt.x + std::cos(currentAngleRad);
    ptOnRobotLine.y = currentPt.y + std::sin(currentAngleRad);
    ptOnRobotLine.z = 0.0;
    double crossProduct = (ptOnRobotLine.y - currentPt.y) * (targetPt.x - currentPt.x) - (ptOnRobotLine.x - currentPt.x) * (targetPt.y - currentPt.y);
    return -sgn(crossProduct); // TODO: check that this actually returns the correct side
}

// Math Functions --------------------------------------------------------------

int PurePursuitController::sgn(double num) {
    if (num < 0) {
        return -1;
    } else {
        return 1;
    }
}

double PurePursuitController::dot(std::vector<double> vec1, std::vector<double> vec2) {
    if (vec1.size() != vec2.size()) {
        return 0;
    }
    int dotProduct = 0;
    for (size_t i = 0; i < vec1.size(); i++) {
        dotProduct += vec1[i] * vec2[i];
    }
    return dotProduct;
}

double PurePursuitController::getAngleFromQuaternion(geometry_msgs::msg::Quaternion q) {
    // returns angle robot is facing in radians
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double PurePursuitController::distanceBetweenPoints(int idx1, int idx2) {
    geometry_msgs::msg::Point pt1 = path.at(idx1), pt2 = path.at(idx2);
    return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
}

} // namespace controller_plugins