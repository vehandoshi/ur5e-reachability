#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/jntarray.hpp>

#include <stdio.h>
#include <iomanip> // For formatting output
#include <iostream>

int main() {
    using namespace KDL;

    Chain ur5e_chain;

    // Link 0 (base to shoulder)
    Joint joint0(Joint::RotZ);
    Frame frame0 = Frame(Vector(0.0, 0.0, 0.0988));
    ur5e_chain.addSegment(Segment(joint0, frame0));

    // Link 1 (shoulder to elbow) — Rotate X -90°
    Joint joint1(Joint::RotZ);
    Frame frame1 = Frame(Rotation::EulerZYX(0.0, 0.0, -M_PI_2)) * 
                   Frame(Vector(0.0, -0.13387, 0.0));
    ur5e_chain.addSegment(Segment(joint1, frame1));

    // Link 2 (elbow to forearm) — Rotate X -90°
    Joint joint2(Joint::RotZ);
    Frame frame2 = Frame(Rotation::EulerZYX(0.0, 0.0, -M_PI_2)) * 
                   Frame(Vector(-0.425, 0.058, 0.0));
    ur5e_chain.addSegment(Segment(joint2, frame2));

    // Link 3 (forearm to wrist1) — Rotate X -90°
    Joint joint3(Joint::RotZ);
    Frame frame3 = Frame(Rotation::EulerZYX(0.0, 0.0, -M_PI_2)) * 
                   Frame(Vector(-0.3922, -0.04, 0.0));
    ur5e_chain.addSegment(Segment(joint3, frame3));

    // Link 4 (wrist1 to wrist2) — Already Z-axis
    Joint joint4(Joint::RotZ);
    Frame frame4 = Frame(Vector(0.0, -0.018, -0.094));
    ur5e_chain.addSegment(Segment(joint4, frame4));

    // Link 5 (wrist2 to wrist3) — Rotate X -90°
    Joint joint5(Joint::RotZ);
    Frame frame5 = Frame(Rotation::EulerZYX(0.0, 0.0, -M_PI_2)) * 
                   Frame(Vector(0.003, -0.048, -0.01));
    ur5e_chain.addSegment(Segment(joint5, frame5));

    std::cout << "UR5e KDL chain built successfully with all joints rotating about Z-axis.\n";

    return 0;
}
