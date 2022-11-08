#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include "std_msgs/String.h"
#include <eigen_conversions/eigen_msg.h>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/macros/class_forward.h>
#include <cmath>
#include <iostream>


class BaseMotionPlanner {

public:
    BaseMotionPlanner(std::string name_in,std::shared_ptr<ros::NodeHandle> nh_);
    ~BaseMotionPlanner();
    geometry_msgs::Pose getCurrentPose();
protected:
    std::string move_group_name;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    const robot_state::JointModelGroup* joint_model_group;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> my_plan;
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    std::shared_ptr<ros::NodeHandle> nh;

    std::vector<double> start_positions;
    geometry_msgs::Pose origin;

    virtual void generateMotion()=0;
    bool plan(std::vector<geometry_msgs::Pose>  &pose);
    bool execute();
    bool checkEERotation();
    bool goHomePosition();
    bool goJointPosition(std::vector<double> jointGoals);
    double deg2rad(double deg){return (deg*M_PI/180.0);}
};

class CameraIntrinsicMotionPlanner : public BaseMotionPlanner{

public:
    CameraIntrinsicMotionPlanner(std::string name_in,std::shared_ptr<ros::NodeHandle> nh_);
private:
    int samplePerPoint, pointPerSide, pointPerHeight;
    std::vector<Eigen::Quaterniond> orientations;
    std::vector<double> sidePositions;
    std::vector<double> heightPositions;

    void generateMotion();
};

class HandEyeMotionPlanner : public BaseMotionPlanner{

public:
    HandEyeMotionPlanner(std::string name_in,std::shared_ptr<ros::NodeHandle> nh_);
private:
    int numPointsPerVertArc, numVertArc, numRadius;
    std::vector<double> vertDegreeList;
    std::vector<double> horiDegreeList;
    std::vector<double> radiusList;

    void generateMotion();
};

class LaserCamMotionPlanner : public BaseMotionPlanner{

public:
    LaserCamMotionPlanner(std::string name_in, std::shared_ptr<ros::NodeHandle> nh_);
private:
    int numLinePerPlane, numPlaneAngles, numPlanePerCircle;
    std::vector<double> vertDegreeList;
    std::vector<double> horiDegreeList;
    std::vector<double> radiusList;

    void generateMotion();
};
