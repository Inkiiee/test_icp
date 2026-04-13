#ifndef __TEST_ICP__ODOM_RECEIVE_NODE_HPP__
#define __TEST_ICP__ODOM_RECEIVE_NODE_HPP__

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "bridge.h"

class OdomLoader: public rclcpp::Node{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
    Bridge * bridge_;

    void received(nav_msgs::msg::Odometry::UniquePtr msg);
public:
    OdomLoader(Bridge* b, const std::string& node_name="odom_loader");
    virtual ~OdomLoader() = default;
};

#endif