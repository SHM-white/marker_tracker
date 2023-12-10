//
// Created by mijiao on 23-12-7.
//

#ifndef MARKER_TRACKER_MARKER_TRACKER_CONTROLLER_H
#define MARKER_TRACKER_MARKER_TRACKER_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <marker_detector/msg/detect_results.hpp>
#include <robot_serial/msg/aim.hpp>

#include "tracker/tracker.h"

class MarkerTrackerController : public rclcpp::Node {
private:
    std::unordered_map<int, Tracker::SharedPtr> trackers;

    rclcpp::Publisher<robot_serial::msg::Aim>::SharedPtr aimPublisher;

    rclcpp::Subscription<marker_detector::msg::DetectResults>::SharedPtr detectResultsSubscription;

    void detectResultsCallback(marker_detector::msg::DetectResults::SharedPtr detectResults);

    int calculateBestTarget(TrackerResults& trackerResults);

public:
    MarkerTrackerController();
};


#endif //MARKER_TRACKER_MARKER_TRACKER_CONTROLLER_H
