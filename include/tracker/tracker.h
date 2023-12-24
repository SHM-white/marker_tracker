//
// Created by mijiao on 23-12-7.
//

#ifndef MARKER_TRACKER_TRACKER_H
#define MARKER_TRACKER_TRACKER_H

#include <memory>

#include <geometry_msgs/msg/pose.hpp>
#include <robot_serial/msg/aim.hpp>
#include <marker_detector/msg/detail/detect_result__struct.hpp>

#include "tracker_result.h"

class Tracker {
protected:
    int id;
public:
    explicit Tracker(int _id);

    virtual robot_serial::msg::Aim track(marker_detector::msg::DetectResult detectResult) = 0;

    using SharedPtr = std::shared_ptr<Tracker>;
};


#endif //MARKER_TRACKER_TRACKER_H
