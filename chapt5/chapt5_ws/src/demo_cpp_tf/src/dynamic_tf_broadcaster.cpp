#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>

using namespace std::chrono_literals;


class DynamicTfBroadcasterNode : public rclcpp::Node
{
public:
  DynamicTfBroadcasterNode()
  : Node("dynamic_tf_broadcaster")
  {
    dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&DynamicTfBroadcasterNode::publish_tf, this));
  }

  void publish_tf()
  {
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;

    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = "map";
    dynamic_transform_stamped.child_frame_id = "base_link";

    dynamic_transform_stamped.transform.translation.x = 2.0;
    dynamic_transform_stamped.transform.translation.y = 3.0;
    dynamic_transform_stamped.transform.translation.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, 30 * M_PI / 180);  // Roll, Pitch, Yaw
    dynamic_transform_stamped.transform.rotation = tf2::toMsg(quat);

    dynamic_broadcaster_->sendTransform(dynamic_transform_stamped);
    RCLCPP_INFO(this->get_logger(), "Published dynamic transform from 'map' to 'base_link'");
  }
private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;      
  rclcpp::TimerBase::SharedPtr timer_;
};  

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicTfBroadcasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}






