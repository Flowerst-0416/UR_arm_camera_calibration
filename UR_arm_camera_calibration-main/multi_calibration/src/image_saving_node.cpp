/**
 * @file image_saving_node.cpp
 * @author Yuchen Wu (wuyc@umich.edu)
 * @brief A node that saves image from camera
 * @version 1.0
 * @date 2022-05-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "std_msgs/String.h"

bool flag = false;
std::string dir;
int imgCount = 0;

/**
 * @brief A callback function for status confirmation
 * 
 * This function will turn on the image saving flag when it 
 * receives confirmation indicating a pose has been reached by
 * the robot.
 * 
 * @param msg 
 */
void status_callback(const std_msgs::String::ConstPtr& msg){
    flag = true;
    ROS_INFO("Image saving node received confirmation!");
}

/**
 * @brief A call backfunction for new image
 * 
 * This function will only proceed if image saving flag is set.
 * If set, this function will save the image to the directory
 * passed in from ROS parameter server.
 * 
 * The confirmation mechanism is to ensure that image is only being
 * saved when the robot has reached a designated pose and is temporarily
 * stationary to guarantee image quality.
 * 
 * 
 * @param im_msg 
 */
void image_callback(const sensor_msgs::ImageConstPtr im_msg) {
    if(flag){
        flag = false;

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(im_msg, sensor_msgs::image_encodings::BGR8);
        
        std::string fileName = dir + "/left-" + std::to_string(imgCount) + ".png";
        imwrite(fileName,cv_ptr->image);
        ROS_INFO("Image %d saved successfully!",imgCount++);
    }
}

/**
 * @brief A main function that starts the node
 * 
 * This function will set up TWO subscriber, one for checking
 * confirmation from trajectory planner and the other one for
 * saving image, if confirmation has been received.
 * 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "image_saving_node");
    ros::NodeHandle nh_;

    dir = argv[1]; 
    std::string topicName;
    if(nh_.hasParam("cameraTopic")){
        nh_.getParam("cameraTopic",topicName);
    }
    else{
        ROS_ERROR("Missing camera topic name, exiting");
        ros::shutdown();
    }

    ros::Subscriber img_sub = nh_.subscribe(topicName,10,image_callback);
    ros::Subscriber status_sub = nh_.subscribe("execution_status",10,status_callback);

    while (true) {
        ros::spinOnce();
    }
    return 0;
}
