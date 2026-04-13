#include "sensor_receive_node.hpp"

#include <functional>
#include <cmath>
#include <vector>

using namespace std::placeholders;

LaserScan::LaserScan(Bridge* b, const std::string& node_name):Node(node_name.c_str()), bridge{b}{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

    this->sub_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = this->sub_cb_group_;

    this->sub_ = this->node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 
        rclcpp::SensorDataQoS(),
        std::bind(&LaserScan::received, this, _1),
        sub_options
    );
}

void LaserScan::received(const std::unique_ptr<sensor_msgs::msg::LaserScan> msg){
    std::vector<double> xs, ys;
    if(!bridge->deskewScan(*msg, xs, ys)){
        double rad = msg->angle_min;
        for(auto r: msg->ranges){
            if(std::isfinite(r) && r >= msg->range_min && r <= msg->range_max){
                xs.push_back(std::cos(rad) * r);
                ys.push_back(std::sin(rad) * r);
            }
            rad += msg->angle_increment;
        }
    }

    bridge->emitScanDataReceived(xs, ys);
}
