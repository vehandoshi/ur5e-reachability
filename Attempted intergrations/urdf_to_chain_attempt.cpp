#include <iostream>
#include <fstream>
#include <sstream>

#include <urdf_model/model.h>
#include "kdl_parser/kdl_parser.hpp"

int main() {
    std::ifstream urdf_file("ur5e.urdf");
    if (!urdf_file) {
        std::cerr << "Failed to open URDF file!" << std::endl;
        return 1;
    }

    std::stringstream buffer;
    buffer << urdf_file.rdbuf();
    std::string urdf_str = buffer.str();

    urdf::Model ur5e_model;
    if (!ur5e_model.initString(urdf_str)) {
        std::cerr << "Failed to parse URDF string!" << std::endl;
        return 1;
    }

    KDL::Tree ur5e_kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(ur5e_model, ur5e_kdl_tree)) {
        std::cerr << "Failed to construct KDL tree!" << std::endl;
        return 1;
    }

    std::cout << "Tree has " << ur5e_kdl_tree.getNrOfSegments() << " segments." << std::endl;

    return 0;
}
