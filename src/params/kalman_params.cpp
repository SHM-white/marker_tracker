//
// Created by mijiao on 23-11-25.
//

#include "params/kalman_params.h"

void KalManParams::init(const rclcpp::Node::SharedPtr _node) {
    node = _node;
    node->declare_parameter("kalman.Dt", 0.005);

    node->declare_parameter("kalman.Qx_gain", 120.0);
    node->declare_parameter("kalman.Qa_gain", 0.1);
    node->declare_parameter("kalman.Rx_gain", 0.0009);
    node->declare_parameter("kalman.Ra_gain", 0.01);
    node->declare_parameter("kalman.Q_x", 0.0);
    node->declare_parameter("kalman.Q_y", 0.0);
    node->declare_parameter("kalman.Q_theta", 0.01);
    node->declare_parameter("kalman.Q_x0", 0.00001);
    node->declare_parameter("kalman.Q_y0", 0.00001);
    node->declare_parameter("kalman.Q_w", 0.01);
    node->declare_parameter("kalman.Q_R", 0.000001);
    node->declare_parameter("kalman.Q_z", 0.01);
    node->declare_parameter("kalman.R_x", 0.002);
    node->declare_parameter("kalman.R_y", 0.002);
    node->declare_parameter("kalman.R_z", 0.0001);
    node->declare_parameter("kalman.R_theta", 0.01);
    node->declare_parameter("kalman.Ra_ang", 0.01);
    node->declare_parameter("kalman.Qa_ang", 0.0001);
    node->declare_parameter("kalman.Qa_a", 0.0000001);
    node->declare_parameter("kalman.Qa_w", 0.0000001);
    node->declare_parameter("kalman.Qa_theta", 0.0);
    node->declare_parameter("kalman.Qa_v", 0.0);

    for (size_t i = 0; i < kalmanPublishers.size(); i++) {
        kalmanPublishers[i] = node->create_publisher<marker_tracker::msg::Kalman>(
                "tracker/t" + std::to_string(i + 1) + "/kalman", 1);
    }
}

double KalManParams::get__Dt() {
    return node->get_parameter("kalman.Dt").as_double();
}

double KalManParams::get__Qx_gain() {
    return node->get_parameter("kalman.Qx_gain").as_double();
}

double KalManParams::get__Qa_gain() {
    return node->get_parameter("kalman.Qa_gain").as_double();
}

double KalManParams::get__Rx_gain() {
    return node->get_parameter("kalman.Rx_gain").as_double();
}

double KalManParams::get__Ra_gain() {
    return node->get_parameter("kalman.Ra_gain").as_double();
}

double KalManParams::get__Q_x() {
    return node->get_parameter("kalman.Q_x").as_double();
}

double KalManParams::get__Q_y() {
    return node->get_parameter("kalman.Q_y").as_double();
}

double KalManParams::get__Q_theta() {
    return node->get_parameter("kalman.Q_theta").as_double();
}

double KalManParams::get__Q_x0() {
    return node->get_parameter("kalman.Q_x0").as_double();
}

double KalManParams::get__Q_y0() {
    return node->get_parameter("kalman.Q_y0").as_double();
}

double KalManParams::get__Q_w() {
    return node->get_parameter("kalman.Q_w").as_double();
}

double KalManParams::get__Q_R() {
    return node->get_parameter("kalman.Q_R").as_double();
}

double KalManParams::get__Q_z() {
    return node->get_parameter("kalman.Q_z").as_double();
}

double KalManParams::get__R_x() {
    return node->get_parameter("kalman.R_x").as_double();
}

double KalManParams::get__R_y() {
    return node->get_parameter("kalman.R_y").as_double();
}

double KalManParams::get__R_z() {
    return node->get_parameter("kalman.R_z").as_double();
}

double KalManParams::get__R_theta() {
    return node->get_parameter("kalman.R_theta").as_double();
}

double KalManParams::get__Ra_ang() {
    return node->get_parameter("kalman.Ra_ang").as_double();
}

double KalManParams::get__Qa_ang() {
    return node->get_parameter("kalman.Qa_ang").as_double();
}

double KalManParams::get__Qa_a() {
    return node->get_parameter("kalman.Qa_a").as_double();
}

double KalManParams::get__Qa_w() {
    return node->get_parameter("kalman.Qa_w").as_double();
}

double KalManParams::get__Qa_theta() {
    return node->get_parameter("kalman.Qa_theta").as_double();
}

double KalManParams::get__Qa_v() {
    return node->get_parameter("kalman.Qa_v").as_double();
}