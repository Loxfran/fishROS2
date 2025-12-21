#include <cstdlib>
#include <ctime>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
using SetP = rcl_interfaces::srv::SetParameters;

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

    std::shared_ptr<SetP::Response> call_set_parameters(
        rcl_interfaces::msg::Parameter &parameter) {
        // 创建客户端,等待服务端上线
        auto param_client = this->create_client<SetP>("/turtle_controler/set_parameters");
        while (!param_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等的服务的过程中被打断！");
                return nullptr;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务上线...");
        }
        //2、构造请求的对象
        auto request = std::make_shared<SetP::Request>();
        request->parameters.push_back(parameter);
        //3、发送异步请求，然后等待响应，响应后调用回调函数
        auto future = param_client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto response = future.get();
        return response;
    };

    void update_server_param_k(double k) {
        //1.创建参数对象
        auto param = rcl_interfaces::msg::Parameter();
        param.name = "k";
        //2.set参数值
        auto param_value = rcl_interfaces::msg::ParameterValue();
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value = k;
        param.value = param_value;
        //3.调用服务端更新参数
        auto response = call_set_parameters(param);
        if (response == nullptr) {
            RCLCPP_WARN(this->get_logger(), "调用服务失败，无法更新参数k");
            return;
        } else {
            for (auto &result : response->results) {
                if (result.successful) {
                    RCLCPP_INFO(this->get_logger(), "参数k更新成功！");
                } else {
                    RCLCPP_WARN(this->get_logger(), "参数k更新失败，原因：%s", result.reason.c_str());
                }
            }
        }
    };
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<PatrolClient>();
    //4.更新参数k
    // client_node->update_server_param_k(0.5);
    rclcpp::spin(client_node);
    rclcpp::shutdown();
    return 0;
};
