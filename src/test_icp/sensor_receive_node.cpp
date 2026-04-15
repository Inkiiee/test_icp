#include "sensor_receive_node.hpp"

#include <cmath>
#include <vector>

LaserScan::LaserScan(Bridge* b, const std::string& node_name)
    : RosSubscriberNode(b, node_name, "/scan") {}

void LaserScan::received(sensor_msgs::msg::LaserScan::UniquePtr msg){
    std::vector<double> xs, ys;
    if(!bridge_->deskewScan(*msg, xs, ys)){
        double rad = msg->angle_min;
        for(auto r: msg->ranges){
            if(std::isfinite(r) && r >= msg->range_min && r <= msg->range_max){
                xs.push_back(std::cos(rad) * r);
                ys.push_back(std::sin(rad) * r);
            }
            rad += msg->angle_increment;
        }
    }

    bridge_->emitScanDataReceived(xs, ys);
}
