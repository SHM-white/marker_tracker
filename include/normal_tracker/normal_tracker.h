//
// Created by mijiao on 23-12-8.
//

#ifndef MARKER_TRACKER_NORMAL_TRACKER_H
#define MARKER_TRACKER_NORMAL_TRACKER_H

#include "tracker/tracker.h"
#include "tracker/tracker_result.h"

class NormalTracker : public Tracker{
private:

public:
    robot_serial::msg::Aim track(geometry_msgs::msg::Pose pose) override;
};


#endif //MARKER_TRACKER_NORMAL_TRACKER_H
