//
// Created by mijiao on 23-12-8.
//

#include "tracker/tracker_result.h"

robot_serial::msg::Aim TrackerResult::toRosMsg() const {
    return robot_serial::msg::Aim()
            .set__pitch(pitch)
            .set__yaw(yaw)
            .set__w_yaw(w_yaw)
            .set__w_pitch(w_pitch)
            .set__target_number(id);
}
