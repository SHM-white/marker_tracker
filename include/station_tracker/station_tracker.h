//
// Created by mijiao on 23-12-8.
//

#ifndef MARKER_TRACKER_BUFF_TRACKER_H
#define MARKER_TRACKER_BUFF_TRACKER_H

#include <Eigen/Core>

#include <marker_detector/msg/detect_results.hpp>
#include "tracker/tracker.h"
#include "ballistics/ballistics.h"
#include "params/cam_params.h"
#include "params/ballistics_params.h"

class StationTracker : public Tracker {
private:
    StationKalman ekf_STATION;
public:
    StationTracker();

    robot_serial::msg::Aim track(marker_detector::msg::DetectResult detectResult) override;

    void reinitialize(std::vector<uint8_t> config) override;
};


#endif //MARKER_TRACKER_BUFF_TRACKER_H
