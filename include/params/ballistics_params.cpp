//
// Created by mijiao on 23-12-10.
//

#include "ballistics_params.h"

void BallisticsParams::init(rclcpp::Node::SharedPtr _node) {
    node = _node;

    node->declare_parameter("ballistics/fire_delay", 0);

    gimbalSubscription = node->create_subscription<robot_serial::msg::Gimbal>(
            "/robot/gimbal",
            1,
            std::bind(
                    &BallisticsParams::gimbalCallback,
                    this,
                    std::placeholders::_1
            )
    );
}

void BallisticsParams::gimbalCallback(const robot_serial::msg::Gimbal::ConstSharedPtr msg) {
    gimbalYaw = msg->yaw * M_PI / 180;
    gimbalPitch = msg->pitch * M_PI / 180;
    gimbalRoll = msg->roll * M_PI / 180;
    ballSpeed = msg->bullet % 128;

    gimbalPitch -= camParams.getPitchOffset();

    cam2world(0, 0) = cos(gimbalYaw) * cos(gimbalPitch);
    cam2world(0, 1) = cos(gimbalYaw) * sin(gimbalPitch) * sin(gimbalRoll) - sin(gimbalYaw) * cos(gimbalRoll);
    cam2world(0, 2) = cos(gimbalYaw) * sin(gimbalPitch) * cos(gimbalRoll) + sin(gimbalYaw) * sin(gimbalRoll);
    cam2world(1, 0) = cos(gimbalPitch) * sin(gimbalYaw);
    cam2world(1, 1) = sin(gimbalYaw) * sin(gimbalPitch) * sin(gimbalRoll) + cos(gimbalYaw) * cos(gimbalRoll);
    cam2world(1, 2) = sin(gimbalYaw) * sin(gimbalPitch) * cos(gimbalRoll) - cos(gimbalYaw) * sin(gimbalRoll);
    cam2world(2, 0) = -sin(gimbalPitch);
    cam2world(2, 1) = sin(gimbalRoll) * cos(gimbalPitch);
    cam2world(2, 2) = cos(gimbalRoll) * cos(gimbalPitch);
}

Eigen::Vector3d BallisticsParams::getWorldCoord(double x, double y, double z) {
    camCoord << -z - camParams.getXOffset(), x, -y + camParams.getZOffset(); //将相机坐标系转为云台坐标
    return cam2world * camCoord;
}

long BallisticsParams::getFireDelay() {
    return node->get_parameter("ballistics/fire_delay").as_int();
}