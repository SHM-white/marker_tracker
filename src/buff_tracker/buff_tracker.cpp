//
// Created by mijiao on 23-12-8.
//

#include "buff_tracker/buff_tracker.h"

BuffTracker::BuffTracker() : Tracker(-1) {}

robot_serial::msg::Aim BuffTracker::track(marker_detector::msg::DetectResult detectResult) {
    robot_serial::msg::Aim aimShoot;
    ballistics.Dafu_aim_shoot(aimShoot,
                              ballisticsParams.getWorldCoord(detectResult.pose.position.x,
                                                             detectResult.pose.position.y,
                                                             detectResult.pose.position.z),
                              ballisticsParams.ballSpeed);
    return aimShoot;
}
