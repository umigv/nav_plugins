#include "CubicBezier.hpp"
#include "PurePursuit.hpp"
#include "DifferentialDrive.hpp"
#include <iostream>

int main(){
    CubicBezier bezier(
        CubicBezier::Knot(0, 0, 0, 2),
        CubicBezier::Knot(4, 2, 0, 2)
    );

    PurePursuit controller(PurePursuit::Gains(3, 6, 1, 0.5));

    controller.setPath(bezier.toDiscretePath(100));

    DifferentialDrive drive(Pose(0, 0, 0), 1);

    while(!controller.isFinished()){
        const Twist twist = controller.step(drive.getState());
        drive.move(twist, 0.01);
        std::cout << drive.getState().X() << " " << drive.getState().Y() << std::endl;
    }
}

