// ws_aruco/src/multi_tracker/src/simple_marker_tracker.cpp
// 개선판 SimpleMarkerTracker + 이미지 오버레이 / 가공영상 퍼블리시 추가

#include <memory>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class SimpleMarkerTracker : public rclcpp::Node
{
public:
  SimpleMarkerTracker()
  : Node("x500_aruco_detector"),
    static_pose_received_(false)
  {
    // ──────────────────────── 파라미터 선언 & 읽기 ────────────────────────
    declare_parameter<std::string>("image_topic",
        "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image");
    declare_parameter<std::string>("pose_static_topic",
        "/model/x500_gimbal_0/pose_static");
    declare_parameter<std::string>("camera_info_topic",
        "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info");
    declare_parameter<std::string>("pose_topic",   "/marker_pose");
    declare_parameter<std::string>("id_topic",     "/marker_id");
    declare_parameter<std::string>("enu_point_topic", "/marker_enu_point");
    declare_parameter<std::string>("image_proc_topic", "/marker/image_proc");
    declare_parameter<int>("aruco_dict_id",        cv::aruco::DICT_4X4_50);
    declare_parameter<double>("marker_size",       0.25);
    declare_parameter<double>("ema_alpha",         1.0);

    get_parameter("image_topic",         image_topic_);
    get_parameter("pose_static_topic",   pose_static_topic_);
    get_parameter("camera_info_topic",   camera_info_topic_);
    get_parameter("pose_topic",          pose_topic_);
    get_parameter("id_topic",            id_topic_);
    get_parameter("enu_point_topic",     enu_point_topic_);
    get_parameter("image_proc_topic",    image_proc_topic_);
    get_parameter("aruco_dict_id",       dict_id_);
    get_parameter("marker_size",         marker_size_);
    get_parameter("ema_alpha",           ema_alpha_);

    // ──────────────────────────── QoS 설정 ────────────────────────────
    auto img_qos  = rclcpp::SensorDataQoS(rclcpp::KeepLast(5));
    auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // ─────────────────────────── 구독자 ────────────────────────────────
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, img_qos,
      std::bind(&SimpleMarkerTracker::imageCb, this, std::placeholders::_1));
    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, pose_qos,
      std::bind(&SimpleMarkerTracker::camInfoCb, this, std::placeholders::_1));
    pose_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      pose_static_topic_, pose_qos,
      std::bind(&SimpleMarkerTracker::poseStaticCb, this, std::placeholders::_1));

    // ───────────────────────── 퍼블리셔 ────────────────────────────────
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, pose_qos);
    id_pub_   = create_publisher<std_msgs::msg::Int32>(id_topic_, pose_qos);
    enu_pub_  = create_publisher<geometry_msgs::msg::PointStamped>(enu_point_topic_, pose_qos);
    img_pub_  = create_publisher<sensor_msgs::msg::Image>(image_proc_topic_, img_qos);

    // ───────────────────────── ArUco 설정 ─────────────────────────────
    dict_   = cv::aruco::getPredefinedDictionary(dict_id_);
    params_ = cv::aruco::DetectorParameters::create();
    params_->cornerRefinementMethod        = cv::aruco::CORNER_REFINE_SUBPIX;
    params_->cornerRefinementWinSize       = 5;
    params_->cornerRefinementMaxIterations = 30;

    RCLCPP_INFO(get_logger(),
      "SimpleMarkerTracker ready (dict=%d, size=%.2f m). Processed images on '%s'",
      dict_id_, marker_size_, image_proc_topic_.c_str());
  }

private:
  // ──────────────────────── 멤버 변수 ─────────────────────────────
  // 구독 & 퍼블리시
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr     pose_static_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr            id_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr enu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         img_pub_;

  // 파라미터 값
  std::string image_topic_, pose_static_topic_, camera_info_topic_;
  std::string pose_topic_, id_topic_, enu_point_topic_, image_proc_topic_;
  int    dict_id_;
  double marker_size_, ema_alpha_;

  // 카메라 보정
  cv::Mat K_, D_;

  // ArUco
  cv::Ptr<cv::aruco::Dictionary>        dict_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;

  // map→camera static transform
  tf2::Transform T_map_camera_;
  bool           static_pose_received_;

  // ──────────────────────── 콜백 함수 ──────────────────────────────
  void camInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!K_.empty()) return;
    K_ = cv::Mat(3,3,CV_64F,const_cast<double*>(msg->k.data())).clone();
    D_ = cv::Mat(msg->d.size(),1,CV_64F,const_cast<double*>(msg->d.data())).clone();
    RCLCPP_INFO(get_logger(),"Camera intrinsics set");
  }

  void poseStaticCb(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto &stamped : msg->transforms) {
      if (stamped.child_frame_id == "x500_gimbal_0") {
        const auto &t = stamped.transform;
        T_map_camera_.setOrigin({t.translation.x, t.translation.y, t.translation.z});
        T_map_camera_.setRotation({t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w});
        static_pose_received_ = true;
        // RCLCPP_INFO(get_logger(),"Received static transform map→camera");
        break;
      }
    }
  }

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!static_pose_received_ || K_.empty()) {
      return; // 필수 정보가 아직 없음
    }

    // CvImage 변환 (항상 BGR8 로 변환)
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // ───── ArUco 검출 ─────
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, dict_, corners, ids, params_);

    // 마커 외곽선 그리기 (검출 실패해도 draw 함수는 안전)
    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

    if (!ids.empty()) {
      // 자세 추정
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, K_, D_, rvecs, tvecs);

      // 카메라 원점(map 좌표)
      const double cx = T_map_camera_.getOrigin().x();
      const double cy = T_map_camera_.getOrigin().y();
      const double cz = T_map_camera_.getOrigin().z();

      for (size_t i = 0; i < ids.size(); ++i) {
        int id = ids[i];

        // camera→marker 변환
        tf2::Transform T_cam_marker = toTf(rvecs[i], tvecs[i]);
        tf2::Vector3 cam_vec = T_cam_marker.getOrigin();

        // 좌표계 맞춤 (x+, y-, z-)
        double out_x = cx + cam_vec.x();
        double out_y = cy - cam_vec.y();
        double out_z = cz - cam_vec.z();

        // ───── 퍼블리시 ─────
        publishData(msg->header.stamp, id, out_x, out_y, out_z, T_cam_marker);

        // ───── 이미지 오버레이 ─────
        cv::aruco::drawAxis(cv_ptr->image, K_, D_, rvecs[i], tvecs[i], marker_size_ * 0.5);

        // 좌표 텍스트
        char buf[128];
        std::snprintf(buf, sizeof(buf), "ID:%d  X:%.2f  Y:%.2f  Z:%.2f", id, out_x, out_y, out_z);
        cv::putText(cv_ptr->image, buf, {10, 30 + static_cast<int>(30*i)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,255}, 2);
      }
    }

    // ───── 가공 영상 퍼블리시 ─────
    sensor_msgs::msg::Image::SharedPtr out_msg = cv_ptr->toImageMsg();
    img_pub_->publish(*out_msg);
  }

  // 데이터 퍼블리시 헬퍼
  void publishData(const rclcpp::Time &stamp, int id,
                   double x, double y, double z,
                   const tf2::Transform &T_cam_marker)
  {
    // PoseStamped (map frame)
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = z;

    tf2::Transform T_map_marker = T_map_camera_ * T_cam_marker;
    pose_msg.pose.orientation = tf2::toMsg(T_map_marker.getRotation());

    pose_pub_->publish(pose_msg);

    // ENU Point
    geometry_msgs::msg::PointStamped pt_msg;
    pt_msg.header = pose_msg.header;
    pt_msg.point.x = x;
    pt_msg.point.y = y;
    pt_msg.point.z = z;
    enu_pub_->publish(pt_msg);

    // ID
    std_msgs::msg::Int32 id_msg; id_msg.data = id;
    id_pub_->publish(id_msg);
  }

  // cv::Vec3d → tf2 변환
  static tf2::Transform toTf(const cv::Vec3d &r, const cv::Vec3d &t)
  {
    cv::Mat R; cv::Rodrigues(r, R);
    tf2::Matrix3x3 tfR(
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
    tf2::Quaternion q; tfR.getRotation(q);
    return tf2::Transform(q, tf2::Vector3(t[0], t[1], t[2]));
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleMarkerTracker>());
  rclcpp::shutdown();
  return 0;
}