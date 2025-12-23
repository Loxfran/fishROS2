#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node
{
public:
  TFListener()
  : Node("tf_listener")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&TFListener::lookup_transform, this));
  }

  void lookup_transform()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        "base_link", "target_point", this->get_clock()->now(),
        rclcpp::Duration::from_seconds(1.0));
      RCLCPP_INFO(
        this->get_logger(),
        "Transform from 'base_link' to 'target_point': Translation(%.2f, %.2f, %.2f)",
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z);
      double yaw, pitch, roll;
      tf2::getEulerYPR(
        transform_stamped.transform.rotation, yaw, pitch, roll);
      RCLCPP_INFO(
        this->get_logger(),
        "Rotation(%.2f, %.2f, %.2f)",yaw, pitch, roll);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform 'map' to 'base_link': %s", ex.what());
    }
  }
private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
