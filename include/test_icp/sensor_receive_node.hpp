#ifndef __TEST_ICP__SENSOR_RECEIVE_NODE_HPP__
#define __TEST_ICP__SENSOR_RECEIVE_NODE_HPP__

#include "ros_subscriber_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScan : public RosSubscriberNode<sensor_msgs::msg::LaserScan, LaserScan> {
public:
    LaserScan(Bridge* b, const std::string& node_name = "laser_scan_reader");
    void received(sensor_msgs::msg::LaserScan::UniquePtr msg);
};

#endif