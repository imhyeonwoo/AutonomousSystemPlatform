#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>   
#include "std_msgs/msg/int32.hpp"

class MultiTrackerNode : public rclcpp::Node
{
public:
    MultiTrackerNode();

private:
    void loadParameters();

    // Callbacks
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
	void annotate_image(cv_bridge::CvImagePtr image);

    // ROS 2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _target_id_pub;

    // Data
	cv::Mat _camera_matrix;
	cv::Mat _dist_coeffs;
	cv::Ptr<cv::aruco::DetectorParameters> _detectorParams;
	cv::Ptr<cv::aruco::Dictionary> _dictionary;

    std::string _image_topic;
    std::string _camera_info_topic;
    std::string _target_id_topic;
    std::string _image_proc_topic;
    std::string _target_pose_topic;

    // State
    std::string _vehicle_type;
    std::array<double, 3> _target;
    int _param_aruco_id {};
	int _param_dictionary {};
	double _param_marker_size {};
};
