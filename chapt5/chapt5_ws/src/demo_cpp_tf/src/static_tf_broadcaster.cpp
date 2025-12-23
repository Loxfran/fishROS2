#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class StaticTfBroadcasterNode : public rclcpp::Node
{
public:
  StaticTfBroadcasterNode()
  : Node("static_tf_broadcaster")
  {
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->publish_tf();
  }

  void publish_tf()
  {
    geometry_msgs::msg::TransformStamped static_transform_stamped;

    static_transform_stamped.header.stamp = this->get_clock()->now();
    static_transform_stamped.header.frame_id = "map";
    static_transform_stamped.child_frame_id = "target_point";

    static_transform_stamped.transform.translation.x = 5.0;
    static_transform_stamped.transform.translation.y = 3.0;
    static_transform_stamped.transform.translation.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, 60 * M_PI / 180);  // Roll, Pitch, Yaw
    static_transform_stamped.transform.rotation = tf2::toMsg(quat);

    static_broadcaster_->sendTransform(static_transform_stamped);
    RCLCPP_INFO(this->get_logger(), "Published static transform from 'map' to 'target_point'");
  }
private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};  

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticTfBroadcasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}






