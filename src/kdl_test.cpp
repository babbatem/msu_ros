#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <stdio.h>
#include <iostream>

using namespace KDL;

int main( int argc, char** argv )
{

    KDL::Tree my_tree;
    if (!kdl_parser::treeFromFile("/home/abba/kinova-ros/kinova_description/urdf/jaco6.urdf", my_tree)){
       // ROS_ERROR("Failed to construct kdl tree");
       printf("(%s)\n", "failed to parse urdf file");
       return false;
    }

    printf("%s\n", "Successfully parsed urdf file");

    //get chain
    KDL::Chain chain;
    std::string base = "world";
    std::string tip = "j2s6s300_link_6";
    bool ret = my_tree.getChain(tip, base, chain);
    if (!ret){
      printf("getChain failed \n");
      return false;
    } else {
      printf("%s\n", "getChain successful. ");
    }

    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);

    // Assign some values to the joint positions
    for(unsigned int i=0;i<nj;i++){
        float myinput;
        printf ("Enter the position of joint %i: ",i);
        scanf ("%e",&myinput);
        jointpositions(i)=(double)myinput;
    }

    // Create the frame that will contain the results
    KDL::Frame cartpos;

    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        printf("%s \n","Success, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }
}
