//
// Created by mijiao on 23-12-7.
//

#ifndef MARKER_TRACKER_TRACKER_H
#define MARKER_TRACKER_TRACKER_H

#include <memory>

#include <Eigen/Core>

#include <geometry_msgs/msg/pose.hpp>
#include <robot_serial/msg/aim.hpp>
#include <marker_detector/msg/detail/detect_result__struct.hpp>

#include "ballistics/ballistics.h"

class Tracker {
protected:
    int id;
    Ballistics ballistics;
public:
    explicit Tracker(int _id);

    virtual ~Tracker() = default;

    virtual robot_serial::msg::Aim track(marker_detector::msg::DetectResult detectResult) = 0;

    using SharedPtr = std::shared_ptr<Tracker>;

    static Eigen::Vector3d toEulerAngles(const geometry_msgs::msg::Quaternion &quaternion);

    virtual void reinitialize(const std::vector<uint8_t> &config);
};

using TrackerResults = std::vector<robot_serial::msg::Aim>;


#endif //MARKER_TRACKER_TRACKER_H
