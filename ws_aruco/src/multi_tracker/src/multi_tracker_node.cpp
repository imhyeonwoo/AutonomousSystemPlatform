// simple_marker_tracker.cpp
//   – State-based marker-size switching
//   – Stores up to 10 unique ArUco IDs (0-9 only), then writes CSV to ~/workspace/ws_PX4

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class SimpleMarkerTracker : public rclcpp::Node
{
public:
  SimpleMarkerTracker()
  : Node("x500_aruco_detector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    /* ---------- parameters ---------- */
    declare_parameter<std::string>("image_topic",
                                   "/world/default/model/x500_gimbal_0/link/"
                                   "camera_link/sensor/camera/image");
    declare_parameter<std::string>("camera_info_topic",
                                   "/world/default/model/x500_gimbal_0/link/"
                                   "camera_link/sensor/camera/camera_info");
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

    current_marker_size_ = default_marker_size_;

    stored_ids_.reserve(10);
    stored_positions_.reserve(10);

    /* ---------- QoS ---------- */
    auto img_qos     = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));
    auto default_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    /* ---------- subscriptions ---------- */
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, img_qos,
        std::bind(&SimpleMarkerTracker::imageCb, this, std::placeholders::_1));

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, default_qos,
        std::bind(&SimpleMarkerTracker::camInfoCb, this, std::placeholders::_1));

    gimbal_mode_sub_ = create_subscription<std_msgs::msg::String>(
        "/gimbal_mode", default_qos,
        std::bind(&SimpleMarkerTracker::gimbalModeCb, this,
                  std::placeholders::_1));

    /* ---------- publishers ---------- */
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_,
                                                                  default_qos);
    id_pub_   = create_publisher<std_msgs::msg::Int32>(id_topic_, default_qos);
    enu_pub_  = create_publisher<geometry_msgs::msg::PointStamped>(
        enu_point_topic_, default_qos);
    text_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        text_marker_topic_, default_qos);
    img_pub_ = create_publisher<sensor_msgs::msg::Image>(image_proc_topic_,
                                                         img_qos);

    dict_   = cv::aruco::getPredefinedDictionary(dict_id_);
    params_ = cv::aruco::DetectorParameters::create();
    params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    RCLCPP_INFO(get_logger(), "Aruco tracker ready (marker %.2fm default).",
                current_marker_size_);
  }

private:
  /* ---------- ROS interfaces ---------- */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        gimbal_mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr            id_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr enu_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr text_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         img_pub_;

  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /* ---------- parameters ---------- */
  std::string image_topic_, camera_info_topic_, camera_frame_, map_frame_;
  std::string pose_topic_, id_topic_, enu_point_topic_,
      text_marker_topic_, image_proc_topic_;
  int    dict_id_;
  double default_marker_size_, final_marker_size_, current_marker_size_;

  /* ---------- camera intrinsic ---------- */
  cv::Mat K_, D_;

  /* ---------- ArUco ---------- */
  cv::Ptr<cv::aruco::Dictionary>        dict_;
  cv::Ptr<cv::aruco::DetectorParameters>params_;

  /* ---------- storage ---------- */
  std::vector<int>          stored_ids_;
  std::vector<tf2::Vector3> stored_positions_;

  /* ---------- helpers ---------- */
  void writeCsv() const
  {
    std::ofstream ofs("/home/ihw/workspace/AutonomousVehiclePlatform/marker_batch.csv");
    ofs << "id,x,y,z\n";
    for (size_t i = 0; i < stored_ids_.size(); ++i) {
      const auto &p = stored_positions_[i];
      ofs << stored_ids_[i] << ',' << p.x() << ',' << p.y() << ',' << p.z()
          << '\n';
    }
    RCLCPP_INFO_STREAM(
        get_logger(),
        "CSV saved to /home/ihw/workspace/AutonomousVehiclePlatform with "
            << stored_ids_.size() << " unique marker poses.");
  }

  /* ---------- callbacks ---------- */
  void gimbalModeCb(const std_msgs::msg::String::SharedPtr msg)
  {
    current_marker_size_ =
        (msg->data == "LOOK_DOWN") ? final_marker_size_ : default_marker_size_;
  }

  void camInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (K_.empty()) {
      K_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
      D_ = cv::Mat(msg->d.size(), 1, CV_64F,
                   const_cast<double *>(msg->d.data()))
               .clone();
      RCLCPP_INFO(get_logger(), "Camera intrinsics received.");
    }
  }

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (K_.empty()) return;

    geometry_msgs::msg::TransformStamped tf_map2cam;
    try {
      tf_map2cam = tf_buffer_.lookupTransform(map_frame_, camera_frame_,
                                              tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN(get_logger(), "TF error: %s", e.what());
      return;
    }

    auto cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, dict_, corners, ids, params_);

    if (!ids.empty()) {
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, current_marker_size_, K_, D_,
                                           rvecs, tvecs);

      for (size_t i = 0; i < ids.size(); ++i) {
        tf2::Vector3 t_cam(-tvecs[i][2], tvecs[i][0], -tvecs[i][1]);
        tf2::Transform T;
        tf2::fromMsg(tf_map2cam.transform, T);
        tf2::Vector3 p_map = T.getOrigin() + T.getBasis() * t_cam;

        publishData(msg->header.stamp, ids[i], p_map);

        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        cv::aruco::drawAxis(cv_ptr->image, K_, D_, rvecs[i], tvecs[i],
                            current_marker_size_ * 0.5);
      }
    }
    img_pub_->publish(*cv_ptr->toImageMsg());
  }

  void publishData(const rclcpp::Time &stamp, int id,
                   const tf2::Vector3 &p_map)
  {
    /* --- PointStamped --- */
    geometry_msgs::msg::PointStamped pt;
    pt.header.stamp    = stamp;
    pt.header.frame_id = map_frame_;
    pt.point.x = p_map.x();
    pt.point.y = p_map.y();
    pt.point.z = p_map.z();
    enu_pub_->publish(pt);

    /* --- ID msg --- */
    std_msgs::msg::Int32 idmsg; idmsg.data = id;
    id_pub_->publish(idmsg);

    /* --- marker text (optional) --- */
    visualization_msgs::msg::Marker label;
    label.header = pt.header;
    label.ns     = "aruco_labels";
    label.id     = id;
    label.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position = pt.point;
    label.pose.position.z += 0.5;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.3;
    label.color.r = label.color.g = 1.0;
    label.color.b = 0.0;
    label.color.a = 1.0;
    label.text =
        "ID:" + std::to_string(id) + "\n(" + std::to_string(p_map.x()) + ", "
        + std::to_string(p_map.y()) + ", " + std::to_string(p_map.z()) + ")";
    label.lifetime = rclcpp::Duration::from_seconds(1.0);
    text_marker_pub_->publish(label);

    /* ---------- store/update (IDs 0-9 only) ---------- */
    if (id < 0 || id > 9) return;          // ★ 저장 제한

    auto it = std::find(stored_ids_.begin(), stored_ids_.end(), id);
    if (it == stored_ids_.end()) {
      if (stored_ids_.size() < 9) {        // 0~8번째까지는 저장만
        stored_ids_.push_back(id);
        stored_positions_.push_back(p_map);
      } else if (stored_ids_.size() == 9) { // 10번째(마지막) ID 저장 후 CSV
        stored_ids_.push_back(id);
        stored_positions_.push_back(p_map);
        writeCsv();
      }
    } else {
      size_t idx = std::distance(stored_ids_.begin(), it);
      stored_positions_[idx] = p_map;      // 좌표 갱신
    }
  }
};

/* ---------- main ---------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleMarkerTracker>();
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
