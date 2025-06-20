#include "multi_tracker/multi_tracker_node.hpp"
#include <sstream>

MultiTrackerNode::MultiTrackerNode()
    : Node("multi_tracker_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting MultiTrackerNode");

    loadParameters();

    RCLCPP_INFO(this->get_logger(), "Loaded vehicle_type: %s", _vehicle_type.c_str());

    // RMW QoS settings
	auto image_qos = rclcpp::QoS(1).best_effort(); 
    auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();  

    _detectorParams = cv::aruco::DetectorParameters::create();
	_dictionary = cv::aruco::getPredefinedDictionary(_param_dictionary);

    _image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        _image_topic, image_qos,
        std::bind(&MultiTrackerNode::image_callback, this, std::placeholders::_1));

    _camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        _camera_info_topic, image_qos,
        std::bind(&MultiTrackerNode::camera_info_callback, this, std::placeholders::_1));

    _target_id_pub = this->create_publisher<std_msgs::msg::Int32>(_target_id_topic, pose_qos);
    _image_pub = this->create_publisher<sensor_msgs::msg::Image>(_image_proc_topic, image_qos);
    _target_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(_target_pose_topic, pose_qos);

}

void MultiTrackerNode::loadParameters()
{
    declare_parameter<std::string>("vehicle_type", "x500");
    declare_parameter<int>("aruco_id", 0);
	declare_parameter<int>("dictionary", 0); // DICT_4X4_50
	declare_parameter<double>("marker_size", 0.5);

    get_parameter("vehicle_type", _vehicle_type);
    get_parameter("aruco_id", _param_aruco_id);
    get_parameter("dictionary", _param_dictionary);
    get_parameter("marker_size", _param_marker_size);

    declare_parameter<std::string>("image_topic", "");
    declare_parameter<std::string>("camera_info_topic", "");
    declare_parameter<std::string>("target_id_topic", "/target_id");
    declare_parameter<std::string>("image_proc_topic", "/image_proc");
    declare_parameter<std::string>("target_pose_topic", "/target_pose");

    get_parameter("image_topic", _image_topic);
    get_parameter("camera_info_topic", _camera_info_topic);
    get_parameter("target_id_topic", _target_id_topic);
    get_parameter("image_proc_topic", _image_proc_topic);
    get_parameter("target_pose_topic", _target_pose_topic);
}

void MultiTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            if (msg->encoding == "rgb8") {
                cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(cv_ptr->image, _dictionary, corners, ids, _detectorParams);
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

        if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {
            std::vector<std::vector<cv::Point2f>> undistortedCorners;

            for (const auto& corner : corners) {
                std::vector<cv::Point2f> undistortedCorner;
                cv::undistortPoints(corner, undistortedCorner, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix);
                undistortedCorners.push_back(undistortedCorner);
            }

            for (size_t i = 0; i < ids.size(); i++) {
                std_msgs::msg::Int32 id_msg;
                id_msg.data = ids[i];
                _target_id_pub->publish(id_msg);

                float half_size = _param_marker_size / 2.0f;
                std::vector<cv::Point3f> objectPoints = {
                    cv::Point3f(-half_size,  half_size, 0),
                    cv::Point3f( half_size,  half_size, 0),
                    cv::Point3f( half_size, -half_size, 0),
                    cv::Point3f(-half_size, -half_size, 0)
                };

                cv::Vec3d rvec, tvec;
                cv::solvePnP(objectPoints, undistortedCorners[i], _camera_matrix, cv::noArray(), rvec, tvec);
                cv::drawFrameAxes(cv_ptr->image, _camera_matrix, cv::noArray(), rvec, tvec, _param_marker_size);

                cv::Mat rot_mat;
                cv::Rodrigues(rvec, rot_mat);

                // 역변환: 마커 기준 카메라 pose
                cv::Mat rot_mat_inv = rot_mat.t();
                cv::Mat tvec_inv = -rot_mat_inv * cv::Mat(tvec);

                // RDF → RFU 변환
                cv::Mat cv_to_ros = (cv::Mat_<double>(3,3) <<
                    1,  0,  0,
                    0, -1,  0,
                    0,  0,  1);

                rot_mat = cv_to_ros * rot_mat_inv;
                cv::Mat tvec_ros_mat = cv_to_ros * tvec_inv;
                cv::Vec3d tvec_ros;
                tvec_ros[0] = tvec_ros_mat.at<double>(0);
                tvec_ros[1] = tvec_ros_mat.at<double>(1);
                tvec_ros[2] = tvec_ros_mat.at<double>(2);

                // 카메라 → 차량 변환 (X1 기준)
                tf2::Transform T_vehicle_to_camera;
                T_vehicle_to_camera.setOrigin(tf2::Vector3(0.45, 0.0, 0.2));  // 차량 기준 카메라 위치
                T_vehicle_to_camera.setRotation(tf2::Quaternion(0, 0, 0, 1)); // 동일 방향 가정

                // 마커 기준 카메라 pose
                tf2::Matrix3x3 tf_rot(
                    rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
                    rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
                    rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2)
                );
                tf2::Quaternion q_cam;
                tf_rot.getRotation(q_cam);

                tf2::Transform T_camera_to_marker;
                T_camera_to_marker.setOrigin(tf2::Vector3(tvec_ros[0], tvec_ros[1], tvec_ros[2]));
                T_camera_to_marker.setRotation(q_cam);

                // 마커 기준 차량 pose
                tf2::Transform T_vehicle_to_marker = T_camera_to_marker * T_vehicle_to_camera.inverse();

                // 차량 좌표계 → RViz 좌표계 (FLU → RFU)
                tf2::Matrix3x3 flu_to_rviz_mat(
                    0, 1, 0,
                    1, 0, 0,
                    0, 0, 1
                );

                tf2::Vector3 pos_FLU = T_vehicle_to_marker.getOrigin();
                tf2::Vector3 pos_RFU = flu_to_rviz_mat * pos_FLU;

                tf2::Matrix3x3 rot_FLU(T_vehicle_to_marker.getRotation());
                tf2::Matrix3x3 rot_RFU = flu_to_rviz_mat * rot_FLU * flu_to_rviz_mat.inverse();

                tf2::Quaternion q_RFU;
                rot_RFU.getRotation(q_RFU);

                geometry_msgs::msg::PoseStamped target_pose;
                // target_pose.header.stamp = msg->header.stamp;
                // target_pose.header.frame_id = "map";

                // target_pose.pose.position.x = pos_RFU.x();
                // target_pose.pose.position.y = pos_RFU.y();
                // target_pose.pose.position.z = pos_RFU.z();

                // target_pose.pose.orientation.x = q_RFU.x();
                // target_pose.pose.orientation.y = q_RFU.y();
                // target_pose.pose.orientation.z = q_RFU.z();
                // target_pose.pose.orientation.w = q_RFU.w();
                target_pose.header.stamp = msg->header.stamp;
                target_pose.header.frame_id = "map";

                target_pose.pose.position.x = pos_RFU.x();
                target_pose.pose.position.y = pos_RFU.y();
                target_pose.pose.position.z = pos_RFU.z();

                target_pose.pose.orientation.x = q_RFU.x();
                target_pose.pose.orientation.y = q_RFU.y();
                target_pose.pose.orientation.z = q_RFU.z();
                target_pose.pose.orientation.w = q_RFU.w();

                _target[0] = pos_RFU.x();
                _target[1] = pos_RFU.y();
                _target[2] = pos_RFU.z();

                _target_pose_pub->publish(target_pose);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Camera intrinsics are not initialized.");
        }

        annotate_image(cv_ptr);

        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = cv_ptr->image;
        _image_pub->publish(*out_msg.toImageMsg().get());

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}



void MultiTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {
		return;
	}

	// Always update the camera matrix and distortion coefficients from the new message
	_camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();   // Use clone to ensure a deep copy
	_dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();   // Use clone to ensure a deep copy

	// Log the first row of the camera matrix to verify correct values
	RCLCPP_INFO(this->get_logger(), "Camera matrix updated:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
		    _camera_matrix.at<double>(0, 0), _camera_matrix.at<double>(0, 1), _camera_matrix.at<double>(0, 2),
		    _camera_matrix.at<double>(1, 0), _camera_matrix.at<double>(1, 1), _camera_matrix.at<double>(1, 2),
		    _camera_matrix.at<double>(2, 0), _camera_matrix.at<double>(2, 1), _camera_matrix.at<double>(2, 2));
	RCLCPP_INFO(this->get_logger(), "Camera Matrix: fx=%f, fy=%f, cx=%f, cy=%f",
		    _camera_matrix.at<double>(0, 0), // fx
		    _camera_matrix.at<double>(1, 1), // fy
		    _camera_matrix.at<double>(0, 2), // cx
		    _camera_matrix.at<double>(1, 2)  // cy
		   );

	// Check if focal length is zero after update
	if (_camera_matrix.at<double>(0, 0) == 0) {
		RCLCPP_ERROR(this->get_logger(), "Focal length is zero after update!");

	} else {
		RCLCPP_INFO(this->get_logger(), "Updated camera intrinsics from camera_info topic.");
	}
}

//cv_bridge 사용
void MultiTrackerNode::annotate_image(cv_bridge::CvImagePtr image)
{
	// Annotate the image with the target position and marker size
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(2);
	stream << "X: "  << _target[0] << " Y: " << _target[1]  << " Z: " << _target[2];
	std::string text_xyz = stream.str();


	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1;
	int thickness = 2;
	int baseline = 0;
	cv::Size textSize = cv::getTextSize(text_xyz, fontFace, fontScale, thickness, &baseline);
	baseline += thickness;
	cv::Point textOrg((image->image.cols - textSize.width - 10), (image->image.rows - 10));
	cv::putText(image->image, text_xyz, textOrg, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 8);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiTrackerNode>());
    rclcpp::shutdown();
    return 0;
}