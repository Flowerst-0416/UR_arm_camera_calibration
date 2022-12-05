
#include "plannerFactory.hpp"

/**
 * @brief Construct a new planner Factory::planner Factory object
 * 
 * @param name_in pass in the MoveIt! manipulator group name
 * @param nh_ pass in a ROS node handle to the class
 * 
 * The above params is stored inside the class and will be used
 * when generating new motion planner
 */
plannerFactory::plannerFactory(std::string name_in,std::shared_ptr<ros::NodeHandle> nh_) : \
    group_name(name_in), nh(nh_) {}

/**
 * @brief a function generate corresponding motion planner from the factory
 * 
 * @param type, the type of motion planner that is being requested
 * @return std::shared_ptr<BaseMotionPlanner>, a pointer of the generated planner
 */
std::shared_ptr<BaseMotionPlanner> plannerFactory::generatePlanner(calibType type){
    switch(type){
        case(camera_intrinsic):
            my_planner = std::make_shared<CameraIntrinsicMotionPlanner> (group_name,nh);
            break;
        case(hand_eye):
            my_planner = std::make_shared<HandEyeMotionPlanner> (group_name,nh);
            break;
        case(laser_cam):
            my_planner = std::make_shared<LaserCamMotionPlanner> (group_name,nh);
            break;
        default:
            ROS_ERROR("Undefined planner type");
            break;
    }
    return my_planner;
}