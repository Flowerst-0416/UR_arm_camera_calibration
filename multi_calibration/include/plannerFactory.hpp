#include "trajectory_planner.hpp"

class plannerFactory{

public:
    enum calibType {
        camera_intrinsic,
        hand_eye,
        laser_cam
    };
    plannerFactory(std::string name_in,std::shared_ptr<ros::NodeHandle> nh_);
    std::shared_ptr<BaseMotionPlanner> generatePlanner(calibType type);

private:
    std::string group_name;
    std::shared_ptr<ros::NodeHandle> nh;
    std::shared_ptr<BaseMotionPlanner> my_planner;
};
