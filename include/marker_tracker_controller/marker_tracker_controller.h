//
// Created by mijiao on 23-12-7.
//

#ifndef MARKER_TRACKER_MARKER_TRACKER_CONTROLLER_H
#define MARKER_TRACKER_MARKER_TRACKER_CONTROLLER_H

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <marker_detector/msg/detect_results.hpp>
#include <robot_serial/msg/aim.hpp>
#include <robot_serial/msg/gimbal.hpp>

#include "tracker/tracker.h"
#include "params/kalman_params.h"

class MarkerTrackerController : public rclcpp::Node {
private:
    std::unordered_map<int, Tracker::SharedPtr> trackers;

    rclcpp::Publisher<robot_serial::msg::Aim>::SharedPtr aimPublisher;

    rclcpp::Subscription<marker_detector::msg::DetectResults>::SharedPtr detectResultsSubscription;

    void detectResultsCallback(marker_detector::msg::DetectResults::SharedPtr detectResults);

    static int calculateBestTarget(const TrackerResults& trackerResults);

public:
    MarkerTrackerController();

    void init();
};


#endif //MARKER_TRACKER_MARKER_TRACKER_CONTROLLER_H
