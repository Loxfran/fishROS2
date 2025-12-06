#include <QApplication>
#include <QLabel>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include "status_interface/msg/system_status.hpp"

using SystemStatus = status_interface::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node {
public:
    SysStatusDisplay() : Node("sys_status_display") {
        subscription_ = this->create_subscription<SystemStatus>(
            "system_status", 10,
            [&](const SystemStatus::SharedPtr msg) -> void {
                label_->setText(
                    get_string_from_msg(msg));
            });
        label_ = new QLabel(get_string_from_msg(std::make_shared<SystemStatus>()));
        label_ -> show();
    }

    QString get_string_from_msg(const SystemStatus::SharedPtr msg) {
    std::stringstream show_str;
    show_str    << "========= 系 统 状 态 ========="<< "\n"
                << " 时    间: \t" << msg->stamp.sec << "\ts\n"           
                << " 用 户 名：" << msg->host << "\t\n"
                << " CPU使用率: \t" << msg->cpu_percent << "\t%\n"
                << " 内存使用率: \t" << msg->memory_percent << "\t%\n"
                << " 内存总大小: \t" << msg->memory_total << "\tGB\n"
                << " 可用内存: \t" << msg->memory_avaiable << "\tGB\n"
                << " 网络发送: \t" << msg->net_sent << "\tGB\n"
                << " 网络接收: \t" << msg->net_recv << "\tGB\n"
                << "==========================";
    return QString::fromStdString(show_str.str());
    }

    private:
        rclcpp::Subscription<SystemStatus>::SharedPtr subscription_;
        QLabel* label_;
    
};



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = std::make_shared<SysStatusDisplay>();
    std::thread spin_thread([&]() -> void {
        rclcpp::spin(node);
    });
    spin_thread.detach();
    app.exec();
    rclcpp::shutdown();
    return 0;
}