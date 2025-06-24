#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <deque>
#include <unordered_set>
#include <fstream>
#include <filesystem>

class MarkerLogger : public rclcpp::Node
{
public:
  MarkerLogger() : Node("marker_logger")
  {
    declare_parameter<std::string>("id_topic",    "/marker_id");
    declare_parameter<std::string>("point_topic", "/marker_enu_point");
    declare_parameter<std::string>("csv_path",    defaultCsvPath());

    id_topic_    = get_parameter("id_topic").as_string();
    point_topic_ = get_parameter("point_topic").as_string();
    csv_path_    = get_parameter("csv_path").as_string();

    std::filesystem::create_directories(
        std::filesystem::path(csv_path_).parent_path());
    csv_.open(csv_path_, std::ios::app);
    if(!csv_.is_open())
      throw std::runtime_error("CSV 파일 열기 실패: " + csv_path_);

    id_sub_ = create_subscription<std_msgs::msg::Int32>(
      id_topic_, 10,
      [this](std_msgs::msg::Int32::SharedPtr m){ id_queue_.push_back(m->data); });

    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      point_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MarkerLogger::pointCb, this, std::placeholders::_1));

    trig_pub_ = create_publisher<std_msgs::msg::Bool>("next_waypoint", 10);

    RCLCPP_INFO(get_logger(), "CSV 저장 경로: %s", csv_path_.c_str());
  }

private:
  void pointCb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if(id_queue_.empty()) return;
    int id = id_queue_.front(); id_queue_.pop_front();

    /* 이미 기록된 마커면 무시 */
    if(logged_ids_.count(id)) return;

    csv_ << id << ','
         << msg->point.x << ','
         << msg->point.y << ','
         << msg->point.z << '\n';
    csv_.flush();

    if(csv_){
      logged_ids_.insert(id);                 // 중복 방지용 집합에 저장
      std_msgs::msg::Bool trig; trig.data = true;
      trig_pub_->publish(trig);
      RCLCPP_INFO(get_logger(), "ID=%d 기록 완료 → Trigger 발행", id);
    }else{
      RCLCPP_ERROR(get_logger(), "CSV 기록 오류! Trigger 취소");
    }
  }

  static std::string defaultCsvPath()
  {
    char date[16];
    std::time_t t = std::time(nullptr);
    std::strftime(date, sizeof(date), "%Y%m%d", std::localtime(&t));
    return std::string(std::getenv("HOME")) + "/.ros/aruco_logs/aruco_log_" + date + ".csv";
  }

  /* 멤버 */
  std::string id_topic_, point_topic_, csv_path_;
  std::ofstream csv_;
  std::deque<int> id_queue_;
  std::unordered_set<int> logged_ids_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr id_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trig_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerLogger>());
  rclcpp::shutdown();
  return 0;
}
