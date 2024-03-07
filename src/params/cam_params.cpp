//
// Created by mijiao on 23-12-10.
//

#include "params/cam_params.h"

void CamParams::init(rclcpp::Node::SharedPtr _node) {
    node = _node;

    node->declare_parameter("camera.height_offset", 0.0);
    node->declare_parameter("camera.pitch_offset", 2.0);
    node->declare_parameter("camera.yaw_offset", -0.4);
    node->declare_parameter("camera.x_offset", 0.0);
    node->declare_parameter("camera.z_offset", 0.0);
}

double CamParams::getHeightOffset() {
    return node->get_parameter("camera.height_offset").as_double();
}

double CamParams::getPitchOffset() {
    return node->get_parameter("camera.pitch_offset").as_double();
}

double CamParams::getYawOffset() {
    return node->get_parameter("camera.yaw_offset").as_double();
}

double CamParams::getXOffset() {
    return node->get_parameter("camera.x_offset").as_double();
}

double CamParams::getZOffset() {
    return node->get_parameter("camera.z_offset").as_double();
}
