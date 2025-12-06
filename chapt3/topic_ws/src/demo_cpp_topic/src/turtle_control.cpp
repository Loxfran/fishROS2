#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include "turtlesim/msg/pose.hpp"


using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    double target_x_{1.0};
    double target_y_{1.0};
    double k_{1.0};
    double max_speed_{3.0};

public:
    explicit TurtleControl(const std::string & node_name)
    : Node(node_name)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            std::bind(&TurtleControl::pose_callback, this, std::placeholders::_1)
        );
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr pose)
    {
        //1、获取当前乌龟的位置
        auto current_x = pose->x;
        auto current_y = pose->y;
        auto message = geometry_msgs::msg::Twist();
        RCLCPP_INFO(this->get_logger(), "当前x:%.2f, 当前y:%.2f", current_x, current_y);
        //2、计算与目标点的距离
        auto distance = std::sqrt(std::pow((target_x_ - current_x), 2) + std::pow((target_y_ - current_y), 2));
        RCLCPP_INFO(this->get_logger(), "距离目标点的距离:%.2f", distance);
        //3、计算角度差
        auto angle_to_target = std::atan2(target_y_ - current_y, target_x_ - current_x);
        auto angle_diff = angle_to_target - pose->theta;
        RCLCPP_INFO(this->get_logger(), "角度差:%.2f", angle_diff);
        //4、计算线速度和角速度
        if (distance > 0.1){   
            if (fabs(angle_diff) > 0.1) {
                message.angular.z = fabs(angle_diff);
            }else {
                message.linear.x = k_ * distance;
            }
        }
        //5、限制最大值并发布
        if (message.linear.x > max_speed_) {
            message.linear.x = max_speed_;
        }
        publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleControl>("turtle_control"));
    rclcpp::shutdown();
    return 0;
}