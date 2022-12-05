
#include "ros/ros.h"
#include "plannerFactory.hpp"

/**
 * @brief The main function that brings up the trajectory planner node
 * 
 * In the main function, a planner factory object will be used to generate
 * the corresponding motion planner based on command-line arguments.
 * 
 * command line option 0: Camera Intrinsic Calibration
 * command line option 1: Handeye Calibration
 * command line option 2: Laser-cam Calibration
 * 
 * @param argc
 * @param argv 
 * @return int 
 */

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_planner_node");
    ros::NodeHandle nh_;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::cout<<"hi"<<std::endl;

    static const std::string PLANNING_GROUP = "manipulator";

    plannerFactory factory = plannerFactory(PLANNING_GROUP,std::make_shared<ros::NodeHandle>());
    plannerFactory::calibType type;

    if(atoi(argv[1]) == 0){
        type = plannerFactory::camera_intrinsic;
        std::shared_ptr<BaseMotionPlanner> planner = factory.generatePlanner(type);
    } 
    else if(atoi(argv[1]) == 1){
        type = plannerFactory::hand_eye;
        std::shared_ptr<BaseMotionPlanner> planner = factory.generatePlanner(type);
    }
    else if(atoi(argv[1]) == 2){
        type = plannerFactory::laser_cam;
        std::shared_ptr<BaseMotionPlanner> planner = factory.generatePlanner(type);
    }

    return 0;
}

