//
// Created by mijiao on 23-12-10.
//

#ifndef MARKER_TRACKER_BALLISTICS_PARAMS_H
#define MARKER_TRACKER_BALLISTICS_PARAMS_H

#include <boost/serialization/singleton.hpp>
#include <rclcpp/node.hpp>
#include <robot_serial/msg/gimbal.hpp>
#include <Eigen/Eigen>

#include "cam_params.h"

class BallisticsParams : public boost::serialization::singleton<BallisticsParams> {
private:
    rclcpp::Node::SharedPtr node;

    double gimbalYaw;
    double gimbalPitch;
    double gimbalRoll;
    std::uint8_t ballSpeed;
    Eigen::Matrix3d cam2world;
    Eigen::Vector3d camCoord;

    rclcpp::Subscription<robot_serial::msg::Gimbal>::SharedPtr gimbalSubscription;

public:
    void init(rclcpp::Node::SharedPtr _node);

    void gimbalCallback(const robot_serial::msg::Gimbal::ConstSharedPtr msg);

    Eigen::Vector3d getWorldCoord(double x, double y, double z);

    int getBallSpeed();
};


#define ballisticsParams BallisticsParams::get_mutable_instance()

#endif //MARKER_TRACKER_BALLISTICS_PARAMS_H
