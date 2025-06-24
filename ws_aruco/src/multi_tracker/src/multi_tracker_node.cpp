// simple_marker_tracker.cpp (State-based Marker Size & CSV Logging Version)
//
// 기능:
// 1. Gazebo의 비표준 카메라 좌표계(-X가 전방)를 정확히 반영하여 좌표 변환 수행
// 2. 실시간 TF를 사용하여 짐벌의 움직임을 반영
// 3. 카메라로 Aruco 마커를 인식하여 'map' 기준의 정확한 절대좌표(ENU)를 계산 및 발행
// 4. 감지된 마커 위치에 좌표 텍스트 라벨(Marker)을 RViz에 발행
// 5. /gimbal_mode 토픽을 구독하여 비행 상태에 따라 동적으로 마커 크기를 변경
// 6. 감지된 마커의 ID와 ENU 좌표를 CSV 파일로 저장 (추가된 기능)

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ===== CSV 저장 기능 추가: 필요한 헤더 파일 =====
#include <fstream>      // 파일 스트림
#include <filesystem>   // 파일 시스템 (경로, 디렉토리 생성)
#include <iomanip>      // std::fixed, std::setprecision

class SimpleMarkerTracker : public rclcpp::Node
{
public:
  SimpleMarkerTracker()
  : Node("x500_aruco_detector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 파라미터 선언 및 읽기
    declare_parameter<std::string>("image_topic", "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image");
    declare_parameter<std::string>("camera_info_topic", "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info");
    declare_parameter<std::string>("camera_frame", "x500_gimbal_0/camera_link");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("pose_topic", "/marker_pose");
    declare_parameter<std::string>("id_topic", "/marker_id");
    declare_parameter<std::string>("enu_point_topic", "/marker_enu_point");
    declare_parameter<std::string>("text_marker_topic", "/marker_label");
    declare_parameter<std::string>("image_proc_topic", "/marker/image_proc");
    declare_parameter<int>("aruco_dict_id", cv::aruco::DICT_4X4_50);
    declare_parameter<double>("default_marker_size", 1.0);
    declare_parameter<double>("final_marker_size", 0.5);
    // ===== CSV 저장 기능 추가: 파일 경로 파라미터 선언 =====
    declare_parameter<std::string>("csv_file_path", ""); // 기본값은 비활성화

    get_parameter("image_topic", image_topic_);
    get_parameter("camera_info_topic", camera_info_topic_);
    get_parameter("camera_frame", camera_frame_);
    get_parameter("map_frame", map_frame_);
    get_parameter("pose_topic", pose_topic_);
    get_parameter("id_topic", id_topic_);
    get_parameter("enu_point_topic", enu_point_topic_);
    get_parameter("text_marker_topic", text_marker_topic_);
    get_parameter("image_proc_topic", image_proc_topic_);
    get_parameter("aruco_dict_id", dict_id_);
    get_parameter("default_marker_size", default_marker_size_);
    get_parameter("final_marker_size", final_marker_size_);
    // ===== CSV 저장 기능 추가: 파일 경로 파라미터 읽기 =====
    get_parameter("csv_file_path", csv_file_path_);

    current_marker_size_ = default_marker_size_;

    auto img_qos  = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));
    auto default_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, img_qos, std::bind(&SimpleMarkerTracker::imageCb, this, std::placeholders::_1));
    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, default_qos, std::bind(&SimpleMarkerTracker::camInfoCb, this, std::placeholders::_1));
    gimbal_mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/gimbal_mode", default_qos, std::bind(&SimpleMarkerTracker::gimbalModeCb, this, std::placeholders::_1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, default_qos);
    id_pub_   = create_publisher<std_msgs::msg::Int32>(id_topic_, default_qos);
    enu_pub_  = create_publisher<geometry_msgs::msg::PointStamped>(enu_point_topic_, default_qos);
    text_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(text_marker_topic_, default_qos);
    img_pub_  = create_publisher<sensor_msgs::msg::Image>(image_proc_topic_, img_qos);

    dict_   = cv::aruco::getPredefinedDictionary(dict_id_);
    params_ = cv::aruco::DetectorParameters::create();
    params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    // ===== CSV 저장 기능 추가: 파일 열기 및 헤더 작성 =====
    if (!csv_file_path_.empty()) {
      try {
        std::filesystem::path path(csv_file_path_);
        if (path.has_parent_path()) {
          std::filesystem::create_directories(path.parent_path());
        }
        
        csv_file_.open(csv_file_path_, std::ios::out | std::ios::app);
        if (csv_file_.is_open()) {
          if (csv_file_.tellp() == 0) {
              csv_file_ << "id,e,n,u\n";
          }
          RCLCPP_INFO(this->get_logger(), "Logging marker data to: %s", csv_file_path_.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_file_path_.c_str());
        }
      } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Filesystem error: %s", e.what());
      }
    }

    RCLCPP_INFO(this->get_logger(), "Aruco Absolute Pose Estimator ready. Default marker size: %.2f", current_marker_size_);
  }

  // ===== CSV 저장 기능 추가: 소멸자에서 파일 닫기 =====
  ~SimpleMarkerTracker()
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
      RCLCPP_INFO(this->get_logger(), "CSV file closed: %s", csv_file_path_.c_str());
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gimbal_mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr id_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr enu_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr text_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string image_topic_, camera_info_topic_, camera_frame_, map_frame_;
  std::string pose_topic_, id_topic_, enu_point_topic_, text_marker_topic_, image_proc_topic_;
  int dict_id_;
  double default_marker_size_, final_marker_size_, current_marker_size_;

  // ===== CSV 저장 기능 추가: 파일 스트림 및 경로 멤버 변수 =====
  std::ofstream csv_file_;
  std::string csv_file_path_;

  cv::Mat K_, D_;
  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;

  void gimbalModeCb(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "LOOK_DOWN") {
      if (current_marker_size_ != final_marker_size_) {
        RCLCPP_INFO(get_logger(), "LOOK_DOWN mode detected. Changing marker size to %.2f", final_marker_size_);
        current_marker_size_ = final_marker_size_;
      }
    } else {
      if (current_marker_size_ != default_marker_size_) {
        RCLCPP_INFO(get_logger(), "Exiting LOOK_DOWN mode. Reverting marker size to %.2f", default_marker_size_);
        current_marker_size_ = default_marker_size_;
      }
    }
  }

  void camInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (K_.empty()) {
      K_ = cv::Mat(3,3,CV_64F,const_cast<double*>(msg->k.data())).clone();
      D_ = cv::Mat(msg->d.size(),1,CV_64F,const_cast<double*>(msg->d.data())).clone();
      RCLCPP_INFO(get_logger(),"Camera intrinsics received.");
    }
  }

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (K_.empty()) {
      RCLCPP_WARN_ONCE(get_logger(), "Waiting for camera info...");
      return;
    }

    geometry_msgs::msg::TransformStamped tf_map_to_cam;
    try {
      tf_map_to_cam = tf_buffer_.lookupTransform(map_frame_, camera_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Could not transform %s to %s: %s",
        map_frame_.c_str(), camera_frame_.c_str(), ex.what());
      return;
    }

    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, dict_, corners, ids, params_);
    
    if (!ids.empty()) {
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, current_marker_size_, K_, D_, rvecs, tvecs);

      for (size_t i = 0; i < ids.size(); ++i) {
        tf2::Vector3 t_vec_in_cam_frame(-tvecs[i][2], tvecs[i][0], -tvecs[i][1]);
        
        tf2::Transform T_map_to_cam;
        tf2::fromMsg(tf_map_to_cam.transform, T_map_to_cam);
        
        tf2::Vector3 V_offset_in_map = T_map_to_cam.getBasis() * t_vec_in_cam_frame;
        tf2::Vector3 P_marker_in_map = T_map_to_cam.getOrigin() + V_offset_in_map;

        publishData(msg->header.stamp, ids[i], P_marker_in_map);
        
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        cv::aruco::drawAxis(cv_ptr->image, K_, D_, rvecs[i], tvecs[i], current_marker_size_ * 0.5);
        char buf[128];
        std::snprintf(buf, sizeof(buf), "ID:%d E:%.2f N:%.2f U:%.2f", ids[i],
            P_marker_in_map.x(), P_marker_in_map.y(), P_marker_in_map.z());
        cv::putText(cv_ptr->image, buf, {10, 30 + static_cast<int>(30*i)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,255}, 2);
      }
    }
    img_pub_->publish(*cv_ptr->toImageMsg());
  }

  void publishData(const rclcpp::Time &stamp, int id, const tf2::Vector3 &P_marker_in_map)
  {
    geometry_msgs::msg::PointStamped pt_msg;
    pt_msg.header.stamp = stamp;
    pt_msg.header.frame_id = map_frame_;
    pt_msg.point.x = P_marker_in_map.x();
    pt_msg.point.y = P_marker_in_map.y();
    pt_msg.point.z = P_marker_in_map.z();
    enu_pub_->publish(pt_msg);

    std_msgs::msg::Int32 id_msg; id_msg.data = id;
    id_pub_->publish(id_msg);
    
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = map_frame_;
    text_marker.header.stamp = stamp;
    text_marker.ns = "aruco_labels";
    text_marker.id = id;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = P_marker_in_map.x();
    text_marker.pose.position.y = P_marker_in_map.y();
    text_marker.pose.position.z = P_marker_in_map.z() + 0.5;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.3;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;
    char buf[128];
    std::snprintf(buf, sizeof(buf), "ID: %d\n(%.2f, %.2f, %.2f)", id,
        P_marker_in_map.x(), P_marker_in_map.y(), P_marker_in_map.z());
    text_marker.text = buf;
    text_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    text_marker_pub_->publish(text_marker);

    // ===== CSV 저장 기능 추가: 파일에 데이터 쓰기 =====
    if (csv_file_.is_open()) {
      csv_file_ << id << ","
                << std::fixed << std::setprecision(4) << P_marker_in_map.x() << ","
                << std::fixed << std::setprecision(4) << P_marker_in_map.y() << ","
                << std::fixed << std::setprecision(4) << P_marker_in_map.z() << "\n";
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleMarkerTracker>();
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}