//
// Created by mijiao on 23-12-8.
//

#include "armor_tracker.h"

robot_serial::msg::Aim ArmorTracker::track(marker_detector::msg::DetectResult detectResult) {
    robot_serial::msg::Aim aimShoot;
    auto angles = toEulerAngles(detectResult.pose.orientation);
    double armor_yaw = angles(2);
    double yaw = ballisticsParams.gimbalYaw + armor_yaw + M_PI;
    const int boardNumber = enemyParams.getIsBig(detectResult.id) ? 2 : 4;
    EkfOutput top_ekf_output{};//储存拓展卡尔曼滤波后的结果
    KalmanOutput kf_output{};//储存卡尔曼滤波后的结果
    int IMM_state = IMM_kalman(!detectResult.detect_success,
                               ballisticsParams.getWorldCoord(detectResult.pose.position.x,
                                                              detectResult.pose.position.y,
                                                              detectResult.pose.position.z),
                               yaw,
                               boardNumber,
                               &kalman_CV, &ekf_ROTA, kf_output, top_ekf_output, ballisticsParams.cam2world);
    if (IMM_state == 1) {
        //用示波器显示卡尔曼
//        kalman_scope.header.stamp = ros::Time::now();
//        kalman_scope.kalman_z = kf_output.z;
//        kalman_scope.raw_z = _wordCoord(2);
//        kalman_scope.kalman_y = kf_output.y;
//        kalman_scope.raw_y = _wordCoord(1);
//        kalman_scope.kalman_x = kf_output.x;
//        kalman_scope.raw_x = _wordCoord(0);
//        kalman_scope.sigma = kf_output.sigma;
//        kalman_scope.v_x = kf_output.v_x;
//        kalman_scope.v_y = kf_output.v_y;
//        kalman_scope.v_z = kf_output.v_z;

        if (kf_output.is_success) {
            //弹道重力补偿
            ballistics.KF_aim_shoot(aimShoot, kf_output, ballisticsParams.ballSpeed, ballisticsParams.gimbalYaw,
                                    detectResult.id);
        } else {
            aimShoot.target_number = 0;
            aimShoot.target_rate = 0;
        }
    } else if (IMM_state == 2) {
//        kalman_scope.header.stamp = ros::Time::now();
//        kalman_scope.raw_x = _wordCoord(0);
//        kalman_scope.raw_y = _wordCoord(1);
//        kalman_scope.raw_z = _wordCoord(2);
////        kalman_scope.kalman_x = top_ekf_output.x;
////        kalman_scope.kalman_y = top_ekf_output.y;
//        kalman_scope.kalman_z = top_ekf_output.theta + M_PI;
//        kalman_scope.v_z = top_ekf_output.w;
//        kalman_scope.v_x = top_ekf_output.x0;
//        kalman_scope.v_y = top_ekf_output.y0;
//        //kalman_scope.sigma = top_ekf_output.R_current;
//        kalman_scope.sigma = markSensor->gimbal_yaw + markSensor->armor_yaw + M_PI;
////        kalman_scope.v_x = x;
////        kalman_scope.v_y = y;
////        kalman_scope.v_z = z;
        double trigger_plot;
        double trigger_top;
        double trigger_bottom;
        if (top_ekf_output.is_success) {
            ballistics.EKF_aim_shoot(aimShoot, top_ekf_output, ballisticsParams.ballSpeed,
                                     boardNumber, ballisticsParams.gimbalYaw,
                                     trigger_plot, trigger_top, trigger_bottom,
                                     detectResult.id, ballisticsParams.getFireDelay());//TODO:传入装甲板数量(待测试）
//            //kalman_scope.sigma = trigger_plot;
//            kalman_scope.kalman_x = trigger_top;
//            kalman_scope.kalman_y = trigger_bottom;
        } else {
            aimShoot.target_number = 0;
            aimShoot.target_rate = 0;
        }
    }

    return aimShoot;
}

Eigen::Vector3d ArmorTracker::toEulerAngles(const geometry_msgs::msg::Quaternion& quaternion) {
    Eigen::Vector3d angles;    //yaw pitch roll
    const auto x = quaternion.x;
    const auto y = quaternion.y;
    const auto z = quaternion.z;
    const auto w = quaternion.w;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}
