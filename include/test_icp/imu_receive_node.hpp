#ifndef __TEST_ICP__IMU_RECEIVE_NODE_HPP__
#define __TEST_ICP__IMU_RECEIVE_NODE_HPP__

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "bridge.h"

class ImuLoader: public rclcpp::Node{
private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
    Bridge* bridge_;

    void received(sensor_msgs::msg::Imu::UniquePtr msg);
public:
    explicit ImuLoader(Bridge* b, const std::string& node_name = "imu_loader");
    virtual ~ImuLoader() = default;
};

#endif