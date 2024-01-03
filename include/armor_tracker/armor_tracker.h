//
// Created by mijiao on 23-12-8.
//

#ifndef MARKER_TRACKER_ARMOR_TRACKER_H
#define MARKER_TRACKER_ARMOR_TRACKER_H

#include <Eigen/Core>
#include <marker_detector/msg/detect_results.hpp>
#include <marker_tracker/msg/kalman.hpp>

#include "tracker/tracker.h"
#include "tracker/tracker_result.h"

#include "params/ballistics_params.h"
#include "kalman/kalman.hpp"

class ArmorTracker : public Tracker {
private:
    Kalman kalman_CV;
    ExternKalman ekf_ROTA;
public:
    explicit ArmorTracker(int id);

    robot_serial::msg::Aim track(marker_detector::msg::DetectResult detectResult) override;
};


#endif //MARKER_TRACKER_ARMOR_TRACKER_H
