//
// Created by mijiao on 23-11-25.
//

#include "kalman_params.h"

void KalManParams::init(rclcpp::Node::SharedPtr _node) {
    node = _node;
    node->declare_parameter("/kalman/Dt");

    node->declare_parameter("kalman/Qx_gain");
    node->declare_parameter("kalman/Qa_gain");
    node->declare_parameter("kalman/Rx_gain");
    node->declare_parameter("kalman/Ra_gain");
    node->declare_parameter("kalman/Q_x");
    node->declare_parameter("kalman/Q_y");
    node->declare_parameter("kalman/Q_theta");
    node->declare_parameter("kalman/Q_x0");
    node->declare_parameter("kalman/Q_y0");
    node->declare_parameter("kalman/Q_w");
    node->declare_parameter("kalman/Q_R");
    node->declare_parameter("kalman/Q_z");
    node->declare_parameter("kalman/R_x");
    node->declare_parameter("kalman/R_y");
    node->declare_parameter("kalman/R_z");
    node->declare_parameter("kalman/R_theta");
    node->declare_parameter("kalman/Ra_ang");
    node->declare_parameter("kalman/Qa_ang");
    node->declare_parameter("kalman/Qa_a");
    node->declare_parameter("kalman/Qa_w");
    node->declare_parameter("kalman/Qa_theta");
    node->declare_parameter("kalman/Qa_v");
}

double KalManParams::get__Dt() {
    return node->get_parameter("/kalman/Dt").as_double();
}

double KalManParams::get__Qx_gain() {
    return node->get_parameter("kalman/Qx_gain").as_double();
}

double KalManParams::get__Qa_gain() {
    return node->get_parameter("kalman/Qa_gain").as_double();
}

double KalManParams::get__Rx_gain() {
    return node->get_parameter("kalman/Rx_gain").as_double();
}

double KalManParams::get__Ra_gain() {
    return node->get_parameter("kalman/Ra_gain").as_double();
}

double KalManParams::get__Q_x() {
    return node->get_parameter("kalman/Q_x").as_double();
}

double KalManParams::get__Q_y() {
    return node->get_parameter("kalman/Q_y").as_double();
}

double KalManParams::get__Q_theta() {
    return node->get_parameter("kalman/Q_theta").as_double();
}

double KalManParams::get__Q_x0() {
    return node->get_parameter("kalman/Q_x0").as_double();
}

double KalManParams::get__Q_y0() {
    return node->get_parameter("kalman/Q_y0").as_double();
}

double KalManParams::get__Q_w() {
    return node->get_parameter("kalman/Q_w").as_double();
}

double KalManParams::get__Q_R() {
    return node->get_parameter("kalman/Q_R").as_double();
}

double KalManParams::get__Q_z() {
    return node->get_parameter("kalman/Q_z").as_double();
}

double KalManParams::get__R_x() {
    return node->get_parameter("kalman/R_x").as_double();
}

double KalManParams::get__R_y() {
    return node->get_parameter("kalman/R_y").as_double();
}

double KalManParams::get__R_z() {
    return node->get_parameter("kalman/R_z").as_double();
}

double KalManParams::get__R_theta() {
    return node->get_parameter("kalman/R_theta").as_double();
}

double KalManParams::get__Ra_ang() {
    return node->get_parameter("kalman/Ra_ang").as_double();
}

double KalManParams::get__Qa_ang() {
    return node->get_parameter("kalman/Qa_ang").as_double();
}

double KalManParams::get__Qa_a() {
    return node->get_parameter("kalman/Qa_a").as_double();
}

double KalManParams::get__Qa_w() {
    return node->get_parameter("kalman/Qa_w").as_double();
}

double KalManParams::get__Qa_theta() {
    return node->get_parameter("kalman/Qa_theta").as_double();
}

double KalManParams::get__Qa_v() {
    return node->get_parameter("kalman/Qa_v").as_double();
}