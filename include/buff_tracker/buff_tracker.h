//
// Created by mijiao on 23-12-8.
//

#ifndef MARKER_TRACKER_BUFF_TRACKER_H
#define MARKER_TRACKER_BUFF_TRACKER_H

#include <Eigen/Core>

#include "tracker/tracker.h"
#include "ballistics/ballistics.h"
#include "params/cam_params.h"
#include "params/ballistics_params.h"

class BuffTracker : public Tracker {
private:
    Ballistics ballistics;
public:
    robot_serial::msg::Aim track(marker_detector::msg::DetectResult detectResult) override;
};


#endif //MARKER_TRACKER_BUFF_TRACKER_H
