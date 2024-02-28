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
#include <robot_serial/msg/mode.hpp>

#include "tracker/tracker.h"
#include "armor_tracker/armor_tracker.h"
#include "buff_tracker/buff_tracker.h"
#include "params/ballistics_params.h"
#include "params/cam_params.h"
#include "params/kalman_params.h"

class MarkerTrackerController : public rclcpp::Node {
private:
    enum class Mode : uint8_t {
        AUTO_AIM, BUFF, OUTPOST, NUM
    } mode = Mode::AUTO_AIM;

    std::array<Tracker::SharedPtr, 8> trackers;

    rclcpp::Publisher<robot_serial::msg::Aim>::SharedPtr aimPublisher;

    rclcpp::Subscription<marker_detector::msg::DetectResults>::SharedPtr detectResultsSubscription;

    rclcpp::Subscription<robot_serial::msg::Mode>::SharedPtr modeSubscription;

    rclcpp::TimerBase::SharedPtr kalmanTimer;

    marker_detector::msg::DetectResults::ConstSharedPtr lastDetectResults;

    void detectResultsCallback(const marker_detector::msg::DetectResults::ConstSharedPtr &detectResults);

    void modeCallback(const robot_serial::msg::Mode::ConstSharedPtr &modeMsg);

    void kalmanCallback();

    static int calculateBestTarget(const TrackerResults &trackerResults);

public:
    MarkerTrackerController();

    void init();
};


#endif //MARKER_TRACKER_MARKER_TRACKER_CONTROLLER_H
