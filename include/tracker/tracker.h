//
// Created by mijiao on 23-12-7.
//

#ifndef MARKER_TRACKER_TRACKER_H
#define MARKER_TRACKER_TRACKER_H

#include <memory>

#include <geometry_msgs/msg/pose.hpp>
#include <robot_serial/msg/aim.hpp>

class Tracker {
private:

public:
    virtual robot_serial::msg::Aim track(geometry_msgs::msg::Pose pose) = 0;

    using SharedPtr = std::shared_ptr<Tracker>;
};


#endif //MARKER_TRACKER_TRACKER_H
