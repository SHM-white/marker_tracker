//
// Created by mijiao on 23-12-10.
//

#ifndef MARKER_TRACKER_BALLISTICS_PARAMS_H
#define MARKER_TRACKER_BALLISTICS_PARAMS_H

#include <boost/serialization/singleton.hpp>
#include <rclcpp/node.hpp>

class BallisticsParams : public boost::serialization::singleton<BallisticsParams> {
private:
    rclcpp::Node::SharedPtr node;
public:
    void init(rclcpp::Node::SharedPtr _node);
};


#define ballisticsParams BallisticsParams::get_mutable_instance()

#endif //MARKER_TRACKER_BALLISTICS_PARAMS_H
