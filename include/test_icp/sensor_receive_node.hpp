#ifndef __TEST_ICP__SENSOR_RECEIVE_NODE_HPP__
#define __TEST_ICP__SENSOR_RECEIVE_NODE_HPP__

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "bridge.h"

class LaserScan: public rclcpp::Node{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
    Bridge * bridge;

    void received(const std::unique_ptr<sensor_msgs::msg::LaserScan> msg);
public:
    LaserScan(Bridge* b, const std::string& node_name = "laser_scan_reader");
    virtual ~LaserScan(){}
};

#endif