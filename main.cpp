#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/jntarray.hpp>

#include <iostream>
#include <stdio.h>

#include <fstream>
#include <cmath> 

#ifndef M_PI_2
#define M_PI_2 1.5707963267948966
#endif

//#include <opencv2/opencv.hpp>
//void displayReachabilityMap();


void save_plane_slice(const std::vector<std::tuple<double,double,double,int>>& data, double target_x) {
    std::ofstream out_file("reachability.csv");

    if (!out_file.is_open()) {
        std::cerr << "Could not open file for writing.\n";
        return;
    }

    out_file << "y,z,reachable\n";

    for (const auto& entry : data) {
        double x, y, z;
        int reachable;
        std::tie(x, y, z, reachable) = entry;

        if (std::abs(x - target_x) < 1e-4) { // floating point comparison
            out_file << y << "," << z << "," << reachable << "\n";
        }
    }

    out_file.close();
    std::cout << "CSV file saved!\n";
}


int main() {
    using namespace KDL;

    Chain ur5e_chain;

    // Joint 1
    ur5e_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0.0, M_PI_2, 0.1625, 0.0)));

    // Joint 2
    ur5e_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(-0.425, 0.0, 0.0, 0.0)));

    // Joint 3
    ur5e_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(-0.3922, 0.0, 0.0, 0.0)));

    // Joint 4
    ur5e_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0.0, M_PI_2, 0.1333, 0.0)));

    // Joint 5
    ur5e_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0.0, -M_PI_2, 0.0997, 0.0)));

    // Joint 6
    ur5e_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0.0, 0.0, 0.0996, 0.0)));

    // End effector Tool
    ur5e_chain.addSegment(Segment(Joint(Joint::None), Frame(Vector(0.0, 0.0, 0.2))));

    std::cout << "UR5e KDL chain constructed using official DH parameters.\n"<< std::endl;

    // Define joint angles (in radians)
    JntArray jointpositions = JntArray(ur5e_chain.getNrOfJoints());
    jointpositions(0) =0; 
    jointpositions(1) = 0;
    jointpositions(2) = 0; 
    jointpositions(3) = 0;
    jointpositions(4) = 0; 
    jointpositions(5) = 0;
    jointpositions(6) = 0;


    // Forward kinematics
    ChainFkSolverPos_recursive fksolver(ur5e_chain);

    // Inverse Kinematics
    ChainIkSolverVel_pinv iksolverV(ur5e_chain); // Inverse velocity solver
    ChainIkSolverPos_NR iksolver(ur5e_chain,fksolver, iksolverV,100,1e-4); // Inverse position solver
    
    // Joint Array for IK
    JntArray q_init(ur5e_chain.getNrOfJoints());
    q_init.data.setZero();
    JntArray q_sol(ur5e_chain.getNrOfJoints());
    
   // Define Vector for IK results
   std::vector<std::tuple<double,double,double,int>> reachability_table;
   int ik_result{0};
   int count_max{0};
   double x_max_reach = -0.9;
    
    // Frame Definitions
    for (double x = -1.0; x<=1.0; x +=0.05){
        int count_this{0};
        for(double y = -1.0; y<= 1.0; y += 0.05){

            for(double z = -1.0; z<=1.0; z+=0.05){

                Frame target_pose(Rotation::RotY(-M_PI_2),Vector(x,y,z));

                ik_result = iksolver.CartToJnt(q_init,target_pose,q_sol);

                if(ik_result>=0){reachability_table.emplace_back(x,y,z,1); count_this+=1;}
                else{reachability_table.emplace_back(x,y,z,0);}   
            }
        }
        if(count_this>count_max){count_max=count_this;x_max_reach = x;}
    }
    
    std::cout<<"Computation Complete\n"<<std::endl;

    std::cout<<"X distance to wall: "<<x_max_reach<<"\nNumber of points reached: "<<count_max<<std::endl;
// save_plane_slice(reachability_table, -0.15);

    return 0;
}

/* ATTEMPT at OpenCV
void displayReachabilityMap(
    const std::vector<std::tuple<double, double, double, int>>& reachability_table,
    double x_fixed,
    double y_min, double y_max,
    double z_min, double z_max,
    double step_size
) {
    int steps_y = static_cast<int>((y_max - y_min) / step_size) + 1;
    int steps_z = static_cast<int>((z_max - z_min) / step_size) + 1;

    // Create a blank image (rows = z, cols = y)
    cv::Mat image(steps_z, steps_y, CV_8UC3, cv::Scalar(255, 255, 255));

    for (const auto& point : reachability_table) {
        double x, y, z;
        int reachable;
        std::tie(x, y, z, reachable) = point;

        // Only visualize points on the fixed X plane
        if (std::abs(x - x_fixed) > 1e-6)
            continue;

        // Map y,z to image coordinates
        int col = static_cast<int>((y - y_min) / step_size);              // left to right
        int row = static_cast<int>((z_max - z) / step_size);              // top to bottom

        // Make sure index is within bounds
        if (row >= 0 && row < image.rows && col >= 0 && col < image.cols) {
            cv::Vec3b color = reachable ? cv::Vec3b(0, 255, 0) : cv::Vec3b(0, 0, 255); // green or red
            image.at<cv::Vec3b>(row, col) = color;
        }
    }

    // Draw a border around the plot
    cv::rectangle(image, cv::Point(0, 0), cv::Point(image.cols - 1, image.rows - 1), cv::Scalar(0, 0, 0), 1);

    // Enlarge the image for visibility
    cv::Mat enlarged;
    cv::resize(image, enlarged, cv::Size(), 10, 10, cv::INTER_NEAREST);

    // Display
    cv::imshow("Reachability at X = " + std::to_string(x_fixed), enlarged);
    //cv::waitKey(0);
} */