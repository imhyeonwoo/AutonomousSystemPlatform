#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <vector>
#include <string>
#include <memory>

class PoseTFBroadcaster : public rclcpp::Node
{
public:
  PoseTFBroadcaster()
  : Node("pose_tf_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 세 개의 pose_static 토픽 구독
    sub_x1_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/X1_asp/pose_static", 10,
      std::bind(&PoseTFBroadcaster::pose_callback, this, std::placeholders::_1));

    sub_x500_static_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/x500_gimbal_0/pose_static", 10,
      std::bind(&PoseTFBroadcaster::pose_callback, this, std::placeholders::_1));

    sub_x500_dynamic_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/x500_gimbal_0/pose", 10,
      std::bind(&PoseTFBroadcaster::pose_callback, this, std::placeholders::_1));
  }

private:
  void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (auto transform : msg->transforms)
    {
      // 1. frame_id가 "default"인 경우 "map"으로 변경
      if (transform.header.frame_id == "default") {
        transform.header.frame_id = "map";
      }

      // 2. 부모/자식 프레임 처리 (A/B -> B)
    //   std::string full_child = transform.child_frame_id;
    //   size_t slash_pos = full_child.rfind('/');
    //   if (slash_pos != std::string::npos) {
    //     transform.child_frame_id = full_child.substr(slash_pos + 1);
    //   }

      // 3. TF 퍼블리시
      tf_broadcaster_->sendTransform(transform);
    }
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_x1_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_x500_static_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_x500_dynamic_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}