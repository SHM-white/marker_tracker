//
// Created by mijiao on 23-12-8.
//

#include "station_tracker/station_tracker.h"

StationTracker::StationTracker() : Tracker(-2) {}

robot_serial::msg::Aim StationTracker::track(marker_detector::msg::DetectResult detectResult) {
    robot_serial::msg::Aim aimShoot;
    EkfOutput station_ekf_output{};
    auto wordCoord = ballisticsParams.getWorldCoord(detectResult.pose.position.x,
                                                    detectResult.pose.position.y,
                                                    detectResult.pose.position.z);
    Eigen::VectorXd ekf_input(4);
    ekf_input(0) = -wordCoord(0);
    ekf_input(1) = -wordCoord(1);//ekf坐标系转换
    ekf_input(2) = wordCoord(2);
    auto angles = toEulerAngles(detectResult.pose.orientation);
    double armorYaw = angles(2);
    ekf_input(3) = ballisticsParams.gimbalYaw + armorYaw + M_PI;

    station_ekf_output = ekf_STATION.extern_kalman_filter(!detectResult.detect_success, ekf_input);
    ballistics.EKF_aim_shoot_forHero(aimShoot, station_ekf_output, ballisticsParams.ballSpeed);
    if (detectResult.detect_success) {
        aimShoot.target_number = 0;
    } else {
        aimShoot.target_number = 1;
    }
    return aimShoot;
}

void StationTracker::reinitialize(const std::vector<uint8_t> &config) {
    ekf_STATION.reset_state();//重新初始化
    ekf_STATION.set_omiga(config[0], true);
}
