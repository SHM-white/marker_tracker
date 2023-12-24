//
// Created by mijiao on 23-12-8.
//

#ifndef MARKER_TRACKER_ARMOR_TRACKER_H
#define MARKER_TRACKER_ARMOR_TRACKER_H

#include <Eigen/Core>
#include <marker_detector/msg/detail/detect_result__struct.hpp>

#include "tracker/tracker.h"
#include "tracker/tracker_result.h"

#include "params/ballistics_params.h"
#include "params/enemy_params.h"
#include "kalman/kalman.hpp"
#include "ballistics/ballistics.h"

class ArmorTracker : public Tracker {
private:
    Kalman kalman_CV;
    ExternKalman ekf_ROTA;
    Ballistics ballistics;
public:
    robot_serial::msg::Aim track(marker_detector::msg::DetectResult detectResult) override;

    static Eigen::Vector3d toEulerAngles(const geometry_msgs::msg::Quaternion& quaternion);
};


#endif //MARKER_TRACKER_ARMOR_TRACKER_H
