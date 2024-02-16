//
// Created by mijiao on 23-12-8.
//

#include "armor_tracker/armor_tracker.h"

ArmorTracker::ArmorTracker(int id) : Tracker(id), kalman_CV(Kalman::KalmanType::CAMODE) {}

robot_serial::msg::Aim ArmorTracker::track(marker_detector::msg::DetectResult detectResult) {
    robot_serial::msg::Aim aimShoot;
    marker_tracker::msg::Kalman kalman_scope;
    auto angles = toEulerAngles(detectResult.pose.orientation);
    double armorYaw = angles(2);
    double yaw = ballisticsParams.gimbalYaw + armorYaw + M_PI;
    const int boardNumber = detectResult.is_big ? 2 : 4;
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
        kalman_scope.header.stamp = rclcpp::Clock().now();


        kalman_scope.kalman_x = kf_output.x;
        kalman_scope.kalman_y = kf_output.y;
        kalman_scope.kalman_z = kf_output.z;
        kalman_scope.raw_x = ballisticsParams.getWorldCoord(detectResult.pose.position.x, detectResult.pose.position.y,
                                                            detectResult.pose.position.z)(0);
        kalman_scope.raw_y = ballisticsParams.getWorldCoord(detectResult.pose.position.x, detectResult.pose.position.y,
                                                            detectResult.pose.position.z)(1);
        kalman_scope.raw_z = ballisticsParams.getWorldCoord(detectResult.pose.position.x, detectResult.pose.position.y,
                                                            detectResult.pose.position.z)(2);
        kalman_scope.sigma = kf_output.sigma;
        kalman_scope.v_x = kf_output.v_x;
        kalman_scope.v_y = kf_output.v_y;
        kalman_scope.v_z = kf_output.v_z;

        if (kf_output.is_success) {
            //弹道重力补偿
            ballistics.KF_aim_shoot(aimShoot, kf_output, ballisticsParams.ballSpeed, ballisticsParams.gimbalYaw,
                                    detectResult.id);
        } else {
            aimShoot.target_number = 0;
            aimShoot.target_rate = 0;
        }
        kalManParams.kalmanPublishers[id - 1]->publish(kalman_scope);
    } else if (IMM_state == 2) {
        kalman_scope.header.stamp = rclcpp::Clock().now();
        kalman_scope.raw_x = ballisticsParams.getWorldCoord(detectResult.pose.position.x, detectResult.pose.position.y,
                                                            detectResult.pose.position.z)(0);
        kalman_scope.raw_y = ballisticsParams.getWorldCoord(detectResult.pose.position.x, detectResult.pose.position.y,
                                                            detectResult.pose.position.z)(1);
        kalman_scope.raw_z = ballisticsParams.getWorldCoord(detectResult.pose.position.x, detectResult.pose.position.y,
                                                            detectResult.pose.position.z)(2);
        kalman_scope.kalman_x = top_ekf_output.x;
        kalman_scope.kalman_y = top_ekf_output.y;
        kalman_scope.kalman_z = top_ekf_output.theta + M_PI;
        kalman_scope.v_z = top_ekf_output.w;
        kalman_scope.v_x = top_ekf_output.x0;
        kalman_scope.v_y = top_ekf_output.y0;
        //kalman_scope.sigma = top_ekf_output.R_current;
        kalman_scope.sigma = (float) yaw;
//        kalman_scope.v_x = x;
//        kalman_scope.v_y = y;
//        kalman_scope.v_z = z;
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
        kalManParams.kalmanPublishers[id - 1]->publish(kalman_scope);
    }

    return aimShoot;
}
