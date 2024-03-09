//
// Created by mijiao on 23-11-25.
//

#ifndef MARKER_DETECTOR_BUFF_PARAMS_H
#define MARKER_DETECTOR_BUFF_PARAMS_H


#include <boost/serialization/singleton.hpp>

#include <rclcpp/rclcpp.hpp>
#include <marker_tracker/msg/kalman.hpp>

class KalManParams : public boost::serialization::singleton<KalManParams> {
private:
    rclcpp::Node::SharedPtr node;
public:
    std::array<rclcpp::Publisher<marker_tracker::msg::Kalman>::SharedPtr, 8> kalmanPublishers;

    void init(rclcpp::Node::SharedPtr _node);

    double get__Dt();

    double get__Qx_gain();

    double get__Qa_gain();

    double get__Rx_gain();

    double get__Ra_gain();

    double get__Q_x();

    double get__Q_y();

    double get__Q_theta();

    double get__Q_x0();

    double get__Q_y0();

    double get__Q_w();

    double get__Q_R();

    double get__Q_z();

    double get__R_x();

    double get__R_y();

    double get__R_z();

    double get__R_theta();

    double get__Ra_ang();

    double get__Qa_ang();

    double get__Qa_a();

    double get__Qa_w();

    double get__Qa_theta();

    double get__Qa_v();
};


#define kalManParams KalManParams::get_mutable_instance()

#endif //MARKER_DETECTOR_BUFF_PARAMS_H
