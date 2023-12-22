//
// Created by mijiao on 23-12-8.
//

#include "buff_tracker.h"

robot_serial::msg::Aim BuffTracker::track(geometry_msgs::msg::Pose pose) {
    robot_serial::msg::Aim aimShoot;
    ballistics.Dafu_aim_shoot(aimShoot,
                              ballisticsParams.getWorldCoord(pose.position.x, pose.position.y, pose.position.z),
                              ballisticsParams.getBallSpeed());
    return aimShoot;
}
