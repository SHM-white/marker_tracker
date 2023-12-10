//
// Created by mijiao on 23-12-8.
//

#ifndef MARKER_TRACKER_TRACKER_RESULT_H
#define MARKER_TRACKER_TRACKER_RESULT_H

#include <vector>

#include <robot_serial/msg/aim.hpp>

struct TrackerResult {
    float pitch;
    float yaw;
    float w_pitch;
    float w_yaw;
    int id;

    bool success;

    [[nodiscard]] robot_serial::msg::Aim toRosMsg() const;
};

using TrackerResults = std::vector<TrackerResult>;

#endif //MARKER_TRACKER_TRACKER_RESULT_H
