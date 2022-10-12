/**
 * @file image_rect_node.cpp
 * @author Yuchen Wu (wuyc@umich.edu)
 * @brief A node undistorts camera image
 * @version 1.0
 * @date 2022-05-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "std_msgs/String.h"
#include <opencv2/core/eigen.hpp>
#include "camodocal/camera_models/CameraFactory.h"

cv::Mat rect_map1_, rect_map2_, R_rect_cv_;  // These matrices will be filled when node was started.
image_transport::Publisher img_pub;
ros::Publisher info_pub;
sensor_msgs::CameraInfo info;

/**
 * @brief A callback function for undistorting newly arrived image
 * 
 * This function will be called for each time a new raw image is 
 * published by the camera. Inside this function, the raw image will be
 * undistorted using given camera intrinsics and the rectified image
 * will be published as well.
 * 
 * @param im_msg, the msg containing new raw image
 */
void image_callback(const sensor_msgs::ImageConstPtr im_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(im_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat undistort;

    // This is the key function that performs the undistortion
    cv::remap(cv_ptr->image, undistort, rect_map1_, rect_map2_, cv::INTER_CUBIC);

    // Create a new msg for the rectified image
    cv_bridge::CvImagePtr cv_ptr2(new cv_bridge::CvImage);

    cv_ptr2->encoding = "bgr8";
    cv_ptr2->header.stamp = ros::Time::now();
    cv_ptr2->header.frame_id = cv_ptr->header.frame_id;
    cv_ptr2->image = undistort;
    img_pub.publish(cv_ptr2->toImageMsg());

    // Publish the undistorted image as well the **rectified** camera matrix
    info.header.stamp = cv_ptr2->header.stamp;
    info_pub.publish(info);
}

/**
 * @brief The main function that starts the node
 * 
 * In this function, ONE subscriber and Two publishers will be set up.
 * The subscriber will keep monitor incoming raw camera image, the first
 * publisher will publish the rectified image while the second publisher
 * will publish the rectified camera Info for other component to use.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "image_rect_node");

    // The path to the config is read from command line
    std::string configPath = argv[1];

    // Initialize the ROS publishers and subscribers
    ros::NodeHandle nh_;
    ros::Subscriber img_sub = nh_.subscribe("/ximea_cam/image_raw",1,image_callback);
    info_pub = nh_.advertise<sensor_msgs::CameraInfo>("/blaser_cam/camera_info",1);
    image_transport::ImageTransport it_(nh_);
    img_pub = it_.advertise("/blaser_cam/image_rect_color",1);

    // Generate a camera object to initialize the rect_map in later steps
    camodocal::CameraPtr m_camera_ = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(configPath);

    R_rect_cv_ = cv::Mat::eye(3, 3, CV_32F);
    R_rect_cv_.at<float>(0, 0) = 0.2;
    R_rect_cv_.at<float>(1, 1) = 0.2;

    cv::Mat K_rect;
    K_rect = m_camera_->initUndistortRectifyMap(rect_map1_, rect_map2_, -1.0, -1.0,
      cv::Size(0,0), -1.0, -1.0, R_rect_cv_);

    // Prepare a rectified camera Info for future publish
    std::vector<double> cameraMatrix;
    double fx,fy,cx,cy;

    if(nh_.hasParam("rectCameraMatrix")){
        nh_.getParam("rectCameraMatrix",cameraMatrix);
        fx = cameraMatrix[0];
        fy = cameraMatrix[1];
        cx = cameraMatrix[2];
        cy = cameraMatrix[3];
    }
    else{
        ROS_ERROR("Missing camera matrix, exiting");
        ros::shutdown();
    }

    info.header.frame_id = "tool0";
    info.width = 2448;
    info.height = 1840;
    info.distortion_model = "plumb_bob";
    info.D = std::vector<double> {0,0,0,0,0};
    info.K = boost::array<double, 9> {fx,0,cx,0,fy,cy,0,0,1};
    info.P = boost::array<double, 12> {fx,0,cx,0,0,fy,cy,0,0,0,1,0};

    while (true) {
        ros::spinOnce();
    }
    return 0;
}