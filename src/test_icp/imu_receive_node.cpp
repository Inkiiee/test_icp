#include "imu_receive_node.hpp"

#include <functional>

using namespace std::placeholders;

ImuLoader::ImuLoader(Bridge* b, const std::string& node_name): Node(node_name.c_str()), bridge_{b}{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

    this->sub_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = this->sub_cb_group_;

    this->sub_ = this->node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        rclcpp::SensorDataQoS(),
        std::bind(&ImuLoader::received, this, _1),
        sub_options
    );
}

void ImuLoader::received(sensor_msgs::msg::Imu::UniquePtr msg){
    double stamp = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    bridge_->emitImuHeadingReceived(
        stamp,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
}
