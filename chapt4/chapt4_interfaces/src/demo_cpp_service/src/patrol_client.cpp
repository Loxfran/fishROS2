#include <cstdlib>
#include <ctime>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"

using namespace std::chrono_literals;
using Patrol = chapt4_interfaces::srv::Patrol;

class PatrolClient : public rclcpp::Node {
private:
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    PatrolClient() : Node("patrol_client") {
        patrol_client_ = this->create_client<Patrol>("patrol");
        timer_ = this->create_wall_timer(
            10s, std::bind(&PatrolClient::timer_callback, this));
        srand(time(NULL));
    }

    void timer_callback() {
        //1、等待服务端上线
        while (!patrol_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等的服务的过程中被打断！");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务上线...");
        }
        //2、构造请求的对象
        auto request = std::make_shared<Patrol::Request>();
        request->target_x = rand() % 15;
        request->target_y = rand() % 15;
        RCLCPP_INFO(this->get_logger(), "请求巡逻到目标点(%.2f, %.2f)", request->target_x, request->target_y);
        //3、发送异步请求，然后等待响应，响应后调用回调函数
        patrol_client_->async_send_request(
            request,
            [this](rclcpp::Client<Patrol>::SharedFuture future) {
                auto response = future.get();
                if (response->result == Patrol::Response::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "巡逻请求成功！");
                } else {
                    RCLCPP_INFO(this->get_logger(), "巡逻请求失败，目标点超出范围！");
                }
            }
        );
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<PatrolClient>();
    rclcpp::spin(client_node);
    rclcpp::shutdown();
    return 0;
}