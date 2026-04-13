#include "odom_receive_node.hpp"

#include <functional>

using namespace std::placeholders;

OdomLoader::OdomLoader(Bridge* b, const std::string& node_name): Node(node_name.c_str()), bridge_{b}{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

    this->sub_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = this->sub_cb_group_;

    this->sub_ = this->node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        rclcpp::SensorDataQoS(),
        std::bind(&OdomLoader::received, this, _1),
        sub_options
    );
}

void OdomLoader::received(nav_msgs::msg::Odometry::UniquePtr msg){
    double stamp = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    double rx = msg->pose.pose.orientation.x;
    double ry = msg->pose.pose.orientation.y;
    double rz = msg->pose.pose.orientation.z;
    double rw = msg->pose.pose.orientation.w;

    bridge_->emitOdomDataReceived(stamp, x, y, z, rx, ry, rz, rw);
}
