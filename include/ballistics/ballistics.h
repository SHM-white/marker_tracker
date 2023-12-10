//
// Created by mijiao on 23-12-10.
//

#ifndef MARKER_DETECTOR_BALLISTICS_H
#define MARKER_DETECTOR_BALLISTICS_H

#include <Eigen/Dense>
#include <angles/angles.h>
#include <robot_serial/msg/aim.hpp>
#include "params/cam_params.h"
#include "kalman/kalman.hpp"

#define BALL_SPEED_10 10
#define BALL_SPEED_15 15
#define BALL_SPEED_18 18
#define TOP_MARKER_CENTER_OFFSET_DISTANCE 0.055
#define TOP_MARKER_CENTER_OFFSET_HEIGHT 0.3045 //这两个参数需要测量
#define SMALL_TGT_RADIUS 0.01 //小装甲板击打半径
#define BIG_TGT_RADIUS 0.08 //大装甲板击打半径
#define TRIGGER_DELAY_OFFSET 0.0 //击发延迟在击发时机的补偿，单位是s
#define TRIGGER_DELAY_OFFSET_FLYTIME 0.08 //击发延迟在子弹飞行时间的补偿，单位是s
#define HERO_TRIGGER_DELAY_OFFSET_FLYTIME 0.08 //击发延迟在子弹飞行时间的补偿，单位是s
#define KF_DELAY_OFFSET_FLYTIME 0.02 //KF系统延迟在子弹飞行时间的补偿，单位是s
#define ENEMY_R_TYPICAL 0.2
#define STATION_R 0.2765

//相机安装误差修正
#define HEIGHT_ZERO_OFFSET camParams.getHeightOffset()
#define PITCH_ZERO_OFFSET camParams.getPitchOffset()
#define YAW_ZERO_OFFSET camParams.getYawOffset()

class Ballistics {
public:
    typedef struct {
        float p00;
        float p10;
        float p01;
        float p20;
        float p11;
        float p02;
        float p30;
        float p21;
        float p12;
        float p03;
    } Ballistic_Coefficients_t;

    float ballisticShootingPitch(uint8_t speed, const float& s2, const float& h);

    float ballisticShootingTime(uint8_t speed, const float& s2, const float& h);

    float ballisticShootingYaw(const float& x, const float& y);

    void
    KF_aim_shoot(robot_serial::msg::Aim& _aim_shoot, const KalmanOutput& _kf_output, const std::uint8_t& _ball_speed,
                 const double& _gimbal_yaw, const int& tgt_number);

    void
    EKF_aim_shoot(robot_serial::msg::Aim& _aim_shoot, const EkfOutput& _ekf_output, const std::uint8_t& _ball_speed,
                  const int& board_number, const double& gimbal_yaw, double& _trigger_plot, double& _trigger_top,
                  double& _trigger_bottom, const int& tgt_number, const double& fire_delay);

    void
    Dafu_aim_shoot(robot_serial::msg::Aim& _aim_shoot, const Eigen::Vector3d& _coord, const std::uint8_t& _ball_speed);

    void
    EKF_aim_shoot_forHero(robot_serial::msg::Aim& aim_shoot, const EkfOutput& _ekf_output,
                          const std::uint8_t& _ball_speed);

    void
    EKF_diaoshe_forHero(robot_serial::msg::Aim& aim_shoot, const EkfOutput& _ekf_output,
                        const std::uint8_t& _ball_speed);

    int calc_shoot_rate(const double& w, const double& tgt_r, const double& car_r);

    int
    calc_shoot_rate(const double& tgt_yaw, const double& gimbal_yaw, const double& tgt_distance, const double& tgt_r);

    int
    calc_shoot_rate(const double& tgt_yaw, const double& gimbal_yaw, const double& tgt_distance, const double& tgt_r,
                    const double& sigma);

    int calc_shoot_rate(const double& sigma);

    double set_tgt_radius(const int& board_number, const int& tgt_number);
};


#endif //MARKER_DETECTOR_BALLISTICS_H
