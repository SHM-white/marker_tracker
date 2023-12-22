//
// Created by mijiao on 23-12-10.
//

#ifndef MARKER_TRACKER_CAM_PARAMS_H
#define MARKER_TRACKER_CAM_PARAMS_H


#include <boost/serialization/singleton.hpp>

#include <rclcpp/rclcpp.hpp>

class CamParams : public boost::serialization::singleton<CamParams> {
private:
    rclcpp::Node::SharedPtr node;
public:
    void init(rclcpp::Node::SharedPtr _node);

    double getHeightOffset();

    double getPitchOffset();

    double getYawOffset();

    double getXOffset();

    double getZOffset();
};

#define camParams CamParams::get_mutable_instance()

#endif //MARKER_TRACKER_CAM_PARAMS_H
