//
// Created by mijiao on 23-12-10.
//

#include "ballistics.h"

using Ballistic_Coefficients_t = Ballistics::Ballistic_Coefficients_t;

Ballistic_Coefficients_t Ball_speed_25_time = {0.01626,0.03098,0.0002764,0.001613,0.0002167,
                                               0.01009,-4.4e-05,1.656e-05,-0.0007987,4.406e-05};
Ballistic_Coefficients_t Ball_speed_15_time = {0.02301f,0.05529f,0.002258f,0.001866f,0.0001357f,
                                               0.01626f,0.0f,0.0001965f,-0.001108f,0.0002978f};
Ballistic_Coefficients_t Ball_speed_25_angle = {-0.002708,0.4523,-0.001145,0.004571,0.003363,
                                                0.001887,0.0001523,0.0001772,0.0002688,0.0001827};
Ballistic_Coefficients_t Ball_speed_15_angle = {-0.09182f,1.549f,0.05563f,-0.01646f,-0.01364f,
                                                -0.01167f,0.003645f,0.008018f,0.009194f,0.00503f};
Ballistic_Coefficients_t Ball_speed_18_angle = {-0.03987f,1.065f,0.008682f,0.003831f,0.009955f,
                                                0.00529f,0.0011f,0.001953f,0.001858f,0.001028f};
Ballistic_Coefficients_t BigBall_speed_14_angle = {-0.01804, 1.468, 0.02178, -0.006885,0.01527,
                                                   -0.007534, 0.001603, 0.003386,0.004843, 0.002165};
Ballistic_Coefficients_t BigBall_speed_14_time = {0.02867, 0.05633, 0.002247, 0.002055,0.0005826,
                                                  0.01724, -0.000033, 0.000156,-0.001229, 0.000307};
Ballistic_Coefficients_t BigBall_speed_15_angle = {-0.02702, 1.286, 0.02723, -0.006088, 0.00793,
                                                   -0.007294, 0.001148, 0.002568, 0.003575, 0.001693};
Ballistic_Coefficients_t BigBall_speed_15_time = {0.02459, 0.05596, 0.002179, 0.001222, 0.0003935,
                                                  0.01429, -4.641e-06, 0.0001202, -0.0008614, 0.0002297};
Ballistic_Coefficients_t BigBall_speed_158_angle = { -0.006846, 1.135, 0.00533, 0.0002182, 0.01679,
                                                     -0.0005519, 0.0006175, 0.001074, 0.001668, 0.0006622};
Ballistic_Coefficients_t BigBall_speed_158_time = {0.02122, 0.0519, 0.001091, 0.00168,  0.0007695,
                                                   0.01535, -4.18e-05, 6.124e-05, -0.001192, 0.0001722};//实际弹速15.8左右

double Ballistics::set_tgt_radius(const int & board_number, const int & tgt_number){
    static double last_tgt_radius = 0.0;
    if (tgt_number == 0){
        return last_tgt_radius;
    }
    if(tgt_number == 1){
        last_tgt_radius = 0.05;
        return 0.05;
    }else if(tgt_number == 7){
        last_tgt_radius = 0.06;
    }
    if(board_number >= 3){
        last_tgt_radius = SMALL_TGT_RADIUS;
        return SMALL_TGT_RADIUS;
    }
    else if(board_number == 2){
        last_tgt_radius = BIG_TGT_RADIUS;
        return BIG_TGT_RADIUS;
    }
    else{
        return last_tgt_radius;
    }
}

float Ballistics::ballisticShootingTime(uint8_t speed,const float& s2,const float& h)
{
    float s = sqrtf(s2);
    float s3 = s*s2;
    float h2 = h*h;
    float h3 = h2*h;
    if(speed == 18){
        return Ball_speed_15_time.p00 + Ball_speed_15_time.p10*s + Ball_speed_15_time.p01*h + Ball_speed_15_time.p20*s2 +Ball_speed_15_time.p11*s*h +
               Ball_speed_15_time.p02*h2 + Ball_speed_15_time.p30*s3 + Ball_speed_15_time.p21*s2*h + Ball_speed_15_time.p12*s*h2 + Ball_speed_15_time.p03*h3;
    }else if(speed == 15){
        return Ball_speed_15_time.p00 + Ball_speed_15_time.p10*s + Ball_speed_15_time.p01*h + Ball_speed_15_time.p20*s2 +Ball_speed_15_time.p11*s*h +
               Ball_speed_15_time.p02*h2 + Ball_speed_15_time.p30*s3 + Ball_speed_15_time.p21*s2*h + Ball_speed_15_time.p12*s*h2 + Ball_speed_15_time.p03*h3;
    } else if(speed == 30){
        return Ball_speed_25_time.p00 + Ball_speed_25_time.p10*s + Ball_speed_25_time.p01*h + Ball_speed_25_time.p20*s2 +Ball_speed_25_time.p11*s*h +
               Ball_speed_25_time.p02*h2 + Ball_speed_25_time.p30*s3 + Ball_speed_25_time.p21*s2*h + Ball_speed_25_time.p12*s*h2 + Ball_speed_25_time.p03*h3;
    } else{
        return BigBall_speed_15_time.p00 + BigBall_speed_15_time.p10*s + BigBall_speed_15_time.p01*h + BigBall_speed_15_time.p20*s2 +BigBall_speed_15_time.p11*s*h +
               BigBall_speed_15_time.p02*h2 + BigBall_speed_15_time.p30*s3 + BigBall_speed_15_time.p21*s2*h + BigBall_speed_15_time.p12*s*h2 + BigBall_speed_15_time.p03*h3;
    }

}
float Ballistics::ballisticShootingPitch(uint8_t speed,const float& s2,const float& h)
{
    float s = sqrtf(s2);
    float s3 = s*s2;
    float h2 = h*h;
    float h3 = h2*h;
    if(speed == 18){
        return Ball_speed_18_angle.p00 + Ball_speed_18_angle.p10*s + Ball_speed_18_angle.p01*h + Ball_speed_18_angle.p20*s2 +Ball_speed_18_angle.p11*s*h +
               Ball_speed_18_angle.p02*h2 + Ball_speed_18_angle.p30*s3 + Ball_speed_18_angle.p21*s2*h + Ball_speed_18_angle.p12*s*h2 + Ball_speed_18_angle.p03*h3 + atan2f(h,s)*180/M_PI;
    }else if(speed == 15){
        return Ball_speed_15_angle.p00 + Ball_speed_15_angle.p10*s + Ball_speed_15_angle.p01*h + Ball_speed_15_angle.p20*s2 +Ball_speed_15_angle.p11*s*h +
               Ball_speed_15_angle.p02*h2 + Ball_speed_15_angle.p30*s3 + Ball_speed_15_angle.p21*s2*h + Ball_speed_15_angle.p12*s*h2 + Ball_speed_15_angle.p03*h3 + atan2f(h,s)*180/M_PI;
    } else if(speed == 30){
        return Ball_speed_25_angle.p00 + Ball_speed_25_angle.p10*s + Ball_speed_25_angle.p01*h + Ball_speed_25_angle.p20*s2 +Ball_speed_25_angle.p11*s*h +
               Ball_speed_25_angle.p02*h2 + Ball_speed_25_angle.p30*s3 + Ball_speed_25_angle.p21*s2*h + Ball_speed_25_angle.p12*s*h2 + Ball_speed_25_angle.p03*h3 + atan2f(h,s)*180/M_PI;
    } else{
        return BigBall_speed_15_angle.p00 + BigBall_speed_15_angle.p10*s + BigBall_speed_15_angle.p01*h + BigBall_speed_15_angle.p20*s2 +BigBall_speed_15_angle.p11*s*h +
               BigBall_speed_15_angle.p02*h2 + BigBall_speed_15_angle.p30*s3 + BigBall_speed_15_angle.p21*s2*h + BigBall_speed_15_angle.p12*s*h2 + BigBall_speed_15_angle.p03*h3 + atan2f(h,s)*180/M_PI;
    }
}
float Ballistics::ballisticShootingYaw(const float& x,const float& y)
{
    float yaw = atan2f(y,x)*180/M_PI + 180;//镜头和坐标系是反的
    return yaw > 180 ? yaw - 360 :yaw;
}
void Ballistics::KF_aim_shoot(robot_serial::msg::Aim& _aim_shoot, const KalmanOutput& _kf_output, const std::uint8_t& _ball_speed, const double & _gimbal_yaw, const int & tgt_number){

    float tar_s2 = (pow(_kf_output.x, 2) + pow(_kf_output.y, 2));//距离平方
    float tar_z = (_kf_output.z + HEIGHT_ZERO_OFFSET);//目标高度
    float fly_time = ballisticShootingTime(_ball_speed, tar_s2, tar_z) + KF_DELAY_OFFSET_FLYTIME;
    fly_time = (fly_time > 0.1) ? fly_time : 0;
    float tar_x_predict =
            (_kf_output.x + fly_time * _kf_output.v_x + 0.5f * pow(fly_time, 2) * _kf_output.a_x);//目标速度补偿
    float tar_y_predict =
            (_kf_output.y + fly_time * _kf_output.v_y + 0.5f * pow(fly_time, 2) * _kf_output.a_y);//目标速度补偿
    float tar_s2_predict = pow(tar_x_predict,2) + pow(tar_y_predict,2);
    _aim_shoot.pitch = ballisticShootingPitch(_ball_speed, tar_s2_predict, tar_z) + PITCH_ZERO_OFFSET;//目标重力补偿，射角
    _aim_shoot.yaw = ballisticShootingYaw(tar_x_predict, tar_y_predict) + YAW_ZERO_OFFSET;//目标速度补偿，飞行时间

    float velocity_length = sqrt(pow(_kf_output.v_x, 2) + pow(_kf_output.v_y, 2));
    float position_length = sqrt(pow(_kf_output.x, 2) + pow(_kf_output.y, 2));
    float accelarte_length = sqrt(pow(_kf_output.a_x, 2) + pow(_kf_output.a_y, 2));
    float angle = acos((_kf_output.x * _kf_output.v_x + _kf_output.y * _kf_output.v_y) /
                       (velocity_length * position_length));
    if (_kf_output.x * _kf_output.v_y - _kf_output.y * _kf_output.v_x > 0)//通过向量叉乘判断正负
        _aim_shoot.w_yaw = std::isnan(velocity_length * sin(angle) / position_length) ? 0 : (velocity_length * sin(angle) / position_length);//计算yaw角速度
    else
        _aim_shoot.w_yaw = std::isnan(-velocity_length * sin(angle) / position_length) ? 0 : (-velocity_length * sin(angle) / position_length);//计算yaw角速度
    _aim_shoot.w_pitch = 0;

    _aim_shoot.target_rate = calc_shoot_rate(_kf_output.sigma);
    //_aim_shoot.target_rate = calc_shoot_rate(_aim_shoot.yaw, _gimbal_yaw, tar_s2, 0.06, _kf_output.sigma);
    _aim_shoot.target_number = tgt_number;
}

/**
 * @brief Ballistic::EKF_aim_shoot
 * 装甲板高度使用预测量，跟随模式加入自适应射频的一个重载
 * @param gimbal_yaw 云台yaw角度
 */
void Ballistics::EKF_aim_shoot(robot_serial::msg::Aim& _aim_shoot, const EkfOutput& _ekf_output, const std::uint8_t& _ball_speed,
                               const int & board_number, const double & gimbal_yaw, double &_trigger_plot, double &_trigger_top,
                               double &_trigger_bottom, const int & tgt_number, const double & _fire_delay){

    double fire_delay = _fire_delay;
//    if (tgt_number == 7){
//        fire_delay *= 1.4;
//    }
    bool if_follow = false; //TODO:是否跟随,应当以参数传入

    Eigen::Matrix2d rotMat;
    Eigen::Vector2d transVec;
    Eigen::Vector2d pointVec;
    Eigen::Vector2d pointVec_;

    double tgt_vx = -_ekf_output.w * _ekf_output.R_current * sin(_ekf_output.theta);
    double tgt_vy = _ekf_output.w * _ekf_output.R_current * cos(_ekf_output.theta);
    float tar_s2 = (pow(_ekf_output.x, 2) + pow(_ekf_output.y, 2));//距离平方
    float tar_z;
    float tar_z_current = (_ekf_output.z_current + HEIGHT_ZERO_OFFSET);//目标高度
    float tar_z_next = (_ekf_output.z_next + HEIGHT_ZERO_OFFSET);//目标高度
    float R;
    float R_current = _ekf_output.R_current;
    float R_next = _ekf_output.R_next;
    //float fly_time = ballisticShootingTime(_ball_speed, tar_s2, tar_z) + TRIGGER_DELAY_OFFSET_FLYTIME;
    float fly_time = ballisticShootingTime(_ball_speed, tar_s2, tar_z) + fire_delay;
    fly_time = (fly_time > 0.1) ? fly_time : 0;
    transVec << _ekf_output.x0, _ekf_output.y0;

    float center_yaw = atan2f(_ekf_output.y0,_ekf_output.x0) + M_PI;
    double min_yaw_distance = M_PI_2;
    int choose_marker = 0;
    bool use_current = true;
    bool trigger = false;
    for (int i = 0; i < board_number; i++){
        double temp_yaw = _ekf_output.theta + _ekf_output.w * fly_time + (double)i * 2*M_PI / (float)board_number;
        double diff = angles::shortest_angular_distance(temp_yaw, center_yaw);//装甲板方向指向外侧
        if (abs(diff) < abs(min_yaw_distance)){
            min_yaw_distance = diff;
            choose_marker = i;
            tar_z = use_current ? tar_z_current : tar_z_next;
            R = use_current ? R_current : R_next;
        }
        use_current = !use_current;
    }
    _trigger_plot = -min_yaw_distance * ENEMY_R_TYPICAL;
    _trigger_top = set_tgt_radius(board_number, tgt_number) - TRIGGER_DELAY_OFFSET * ENEMY_R_TYPICAL * _ekf_output.w;
    _trigger_bottom = -set_tgt_radius(board_number, tgt_number) - TRIGGER_DELAY_OFFSET * ENEMY_R_TYPICAL * _ekf_output.w;
    trigger = -min_yaw_distance * ENEMY_R_TYPICAL > -set_tgt_radius(board_number, tgt_number) - TRIGGER_DELAY_OFFSET * ENEMY_R_TYPICAL * _ekf_output.w &&
              -min_yaw_distance * ENEMY_R_TYPICAL < set_tgt_radius(board_number, tgt_number) - TRIGGER_DELAY_OFFSET * ENEMY_R_TYPICAL * _ekf_output.w;

    rotMat << cos(_ekf_output.theta + _ekf_output.w * fly_time + choose_marker * 2*M_PI / (float)board_number ),
            -sin(_ekf_output.theta + _ekf_output.w * fly_time + choose_marker * 2*M_PI / (float)board_number),
            sin(_ekf_output.theta + _ekf_output.w * fly_time + choose_marker * 2*M_PI / (float)board_number),
            cos(_ekf_output.theta + _ekf_output.w * fly_time + choose_marker * 2*M_PI / (float)board_number);
    pointVec << R, 0;
    pointVec_ = rotMat * pointVec + transVec;

    float tar_x_predict = pointVec_[0];
    float tar_y_predict = pointVec_[1];

//    float tar_x_predict = _ekf_output.x0 + _ekf_output.R_current*cos(_ekf_output.theta + _ekf_output.w*fly_time);
//    float tar_y_predict = _ekf_output.y0 + _ekf_output.R_current*sin(_ekf_output.theta + _ekf_output.w*fly_time);

    _aim_shoot.pitch = ballisticShootingPitch(_ball_speed, tar_s2, tar_z) + PITCH_ZERO_OFFSET;//目标重力补偿，射角
    _aim_shoot.yaw = if_follow ? ballisticShootingYaw(tar_x_predict, tar_y_predict) + YAW_ZERO_OFFSET :
                    ballisticShootingYaw(_ekf_output.x0, _ekf_output.y0) + YAW_ZERO_OFFSET;//目标速度补偿，飞行时间

    _aim_shoot.w_pitch = 0;
    float velocity_length = _ekf_output.w * _ekf_output.R_current;
    float position_length = sqrt(pow(_ekf_output.x, 2) + pow(_ekf_output.y, 2));
    float angle = acos((_ekf_output.x * tgt_vx + _ekf_output.y * tgt_vy) /
                       (velocity_length * position_length));
    if (-_ekf_output.y * tgt_vx + _ekf_output.x * tgt_vy > 0)//通过向量叉乘判断正负
        _aim_shoot.w_yaw = std::isnan(velocity_length * sin(angle) / position_length) ? 0 : (velocity_length * sin(angle) / position_length);//计算yaw角速度
    else
        _aim_shoot.w_yaw = std::isnan(-velocity_length * sin(angle) / position_length) ? 0 : (-velocity_length * sin(angle) / position_length);//计算yaw角速度
    if (!if_follow) {
        _aim_shoot.w_yaw = 0;
    }
    if (if_follow){
        _aim_shoot.target_rate = std::isnan(calc_shoot_rate(_aim_shoot.yaw, gimbal_yaw, tar_s2, 0.06)) ? 0 : (calc_shoot_rate(_aim_shoot.yaw, gimbal_yaw, tar_s2, 0.06));
        //aim_shoot.target_rate = 8;
        _aim_shoot.target_number = tgt_number;//识别的装甲板数字
    }
    else{
        _aim_shoot.target_rate = trigger ? calc_shoot_rate(_ekf_output.w, set_tgt_radius(board_number, tgt_number), ENEMY_R_TYPICAL) : 0;
        if (tgt_number == 7 && _aim_shoot.target_rate){
            _aim_shoot.target_rate *= 2;
        }
        _aim_shoot.target_number = 9 + tgt_number;//magic number,don't change!
    }
}


void Ballistics::EKF_aim_shoot_forHero(robot_serial::msg::Aim& aim_shoot, const EkfOutput& _ekf_output, const std::uint8_t& _ball_speed){
    bool shoot_top_marker = false;//TODO：这个值应当以参数传入
    if (!shoot_top_marker)
    {
        Eigen::Matrix2d rotMat;
        Eigen::Vector2d transVec;
        Eigen::Vector2d pointVec;
        Eigen::Vector2d pointVec_;
        transVec << _ekf_output.x0, _ekf_output.y0;
        float tar_s2 = (pow(_ekf_output.x0, 2) + pow(_ekf_output.y0, 2));//到对面中心距离平k方
        tar_s2 = pow(pow(tar_s2, 0.5) - STATION_R, 2);//到装甲板距离平方
        float tar_z = (_ekf_output.z_current + HEIGHT_ZERO_OFFSET);//目标高度
        float fly_time = ballisticShootingTime(_ball_speed, tar_s2, tar_z) + HERO_TRIGGER_DELAY_OFFSET_FLYTIME;
        fly_time = (fly_time > 0.1) ? fly_time : 0;

        float center_yaw = atan2f(_ekf_output.y0, _ekf_output.x0) + M_PI;



        double min_yaw_distance = M_PI_2;
        int choose_marker = 0;
        bool trigger = false;
        for (int i = 0; i < 3; i++){
            double temp_yaw = _ekf_output.theta + _ekf_output.w * fly_time + (double)i * 2*M_PI / 3.0;
            double diff = angles::shortest_angular_distance(temp_yaw, center_yaw);//装甲板方向指向外侧
            if (abs(diff) < abs(min_yaw_distance)){
                min_yaw_distance = diff;
                choose_marker = i;
            }
        }

        trigger = -min_yaw_distance * STATION_R > -set_tgt_radius(3, 7) - TRIGGER_DELAY_OFFSET * STATION_R * _ekf_output.w &&
                  -min_yaw_distance * STATION_R < set_tgt_radius(3, 7) - TRIGGER_DELAY_OFFSET * STATION_R * _ekf_output.w;
        rotMat << cos(_ekf_output.theta + _ekf_output.w * fly_time + choose_marker * 2*M_PI / 3.0),
                -sin(_ekf_output.theta + _ekf_output.w * fly_time + choose_marker * 2*M_PI / 3.0),
                sin(_ekf_output.theta + _ekf_output.w * fly_time + choose_marker * 2*M_PI / 3.0),
                cos(_ekf_output.theta + _ekf_output.w * fly_time + choose_marker * 2*M_PI / 3.0);
        pointVec << STATION_R, 0;
        pointVec_ = rotMat * pointVec + transVec;

        float tar_x_predict = pointVec_[0];
        float tar_y_predict = pointVec_[1];
        aim_shoot.pitch = ballisticShootingPitch(_ball_speed, tar_s2, tar_z) + PITCH_ZERO_OFFSET;//目标重力补偿，射角
        aim_shoot.yaw = ballisticShootingYaw(tar_x_predict, tar_y_predict) + YAW_ZERO_OFFSET;//瞄准中心
        aim_shoot.target_rate = trigger ? calc_shoot_rate(_ekf_output.w, set_tgt_radius(3, 7), STATION_R) : 0;




    }
    else{
        float tar_s2 = (pow(_ekf_output.x0, 2) + pow(_ekf_output.y0, 2));//到对面中心距离平k方
        tar_s2 = pow(pow(tar_s2, 0.5) - TOP_MARKER_CENTER_OFFSET_DISTANCE, 2);
        float tar_z = _ekf_output.z_current + HEIGHT_ZERO_OFFSET + TOP_MARKER_CENTER_OFFSET_HEIGHT;//目标高度
        float tar_x_predict = _ekf_output.x0;
        float tar_y_predict = _ekf_output.y0;
        aim_shoot.pitch = ballisticShootingPitch(_ball_speed, tar_s2, tar_z) + PITCH_ZERO_OFFSET;//目标重力补偿，射角
        aim_shoot.yaw = ballisticShootingYaw(tar_x_predict, tar_y_predict) + YAW_ZERO_OFFSET;//瞄准中心
    }

}
void Ballistics::EKF_diaoshe_forHero(robot_serial::msg::Aim& aim_shoot, const EkfOutput& _ekf_output, const std::uint8_t& _ball_speed){
    float tar_s2 = (pow(_ekf_output.x0, 2) + pow(_ekf_output.y0, 2));
    float tar_z = (_ekf_output.z_current + HEIGHT_ZERO_OFFSET + 0.2);//目标高度
    float fly_time = ballisticShootingTime(_ball_speed, tar_s2, tar_z);
    fly_time = (fly_time > 0.1) ? fly_time : 0;
    aim_shoot.pitch = ballisticShootingPitch(_ball_speed, tar_s2, tar_z) + PITCH_ZERO_OFFSET;//目标重力补偿，射角
    aim_shoot.yaw = ballisticShootingYaw(_ekf_output.x0, _ekf_output.y0) + YAW_ZERO_OFFSET;//瞄准中心
}
void Ballistics::Dafu_aim_shoot(robot_serial::msg::Aim& _aim_shoot, const Eigen::Vector3d& _coord, const std::uint8_t& _ball_speed){
    float s2 = pow(_coord(0),2)+pow(_coord(1),2);

    float _pitch = ballisticShootingPitch(_ball_speed,s2,_coord(2)) + PITCH_ZERO_OFFSET;
    //std::cout<<"adsdsdsdsdsdsdsdsdsdsdsdsdsd"<<_pitch<<std::endl;
    if(std::isnan(_pitch)|| std::isinf(_pitch)) return ;
    float _yaw = ballisticShootingYaw(_coord(0),_coord(1)) + YAW_ZERO_OFFSET;

    _aim_shoot.pitch = 0.2 * _aim_shoot.pitch + 0.8 * _pitch;//低通滤波
    _aim_shoot.yaw = 0.2 * _aim_shoot.yaw + 0.8 * _yaw;

//    _aim_shoot.pitch = _pitch;//低通滤波
//    _aim_shoot.yaw = _yaw;
}

/**
 * @brief Ballistic::calc_shoot_rate
 * 不跟随反陀螺的自动射频
 */
int Ballistics::calc_shoot_rate(const double & w, const double & tgt_r, const double & car_r){
    double rate =  3 * abs((tgt_r / (car_r * w)) / TRIGGER_DELAY_OFFSET_FLYTIME);
    rate = std::isnan(rate) ? 1 : rate;
    if (rate > 0 && rate <= 1) {
        return 10;
    }
    else if (rate > 1){
        return ((int)(rate) * 10 > 40 ? 40 : (int)(rate) * 10);
    }
}

/**
 * @brief Ballistic::calc_shoot_rate
 * 跟随反陀螺的自动射频
 * @param tgt_yaw 向云台发送的角度
 * @param gimbal_yaw 云台当前角度
 * @param tgt_r 靶面半径
 */
int Ballistics::calc_shoot_rate(const double & tgt_yaw, const double & gimbal_yaw, const double & tgt_distance, const double & tgt_r){
    double rate = 10 * tgt_r / (abs(angles::shortest_angular_distance(tgt_yaw, gimbal_yaw)) * pow(tgt_distance, 0.5));
    rate = std::isnan(rate) ? 0 : rate;
    if (rate > 20) {
        return 20;
    }
    else if (rate > 5 && rate <= 20){
        return (int)(rate);
    }
    else if (rate > 0 && rate <= 5){
        return 0;
    }
    else{
        return 0;
    }
}

/**
 * @brief Ballistic::calc_shoot_rate
 * KF考虑云台响应的自动射频
 * @param tgt_yaw 向云台发送的角度
 * @param gimbal_yaw 云台当前角度
 * @param tgt_r 靶面半径
 * @param sigma 卡尔曼误差
 */
int Ballistics::calc_shoot_rate(const double & tgt_yaw, const double & gimbal_yaw, const double & tgt_distance, const double & tgt_r, const double & sigma){
    double rate = 10 * tgt_r / (abs(angles::shortest_angular_distance(tgt_yaw, gimbal_yaw)) * pow(tgt_distance, 0.5));
    rate = std::isnan(rate) ? 0 : rate;
    if (rate > 20) {
        if(sigma < 0.5)
            return 15;
        else if(sigma < 1)
            return 10;
        else if(sigma < 5)
            return (int)(10 - 2.5 * (sigma - 1));
        else
            return 0;
    }
    else if (rate > 5 && rate <= 20){
        if(sigma < 0.5)
            return (int)(15 * rate / 20);
        else if(sigma < 1)
            return 10;
        else if(sigma < 5)
            return (int)((10 - 2.5 * (sigma - 1)) * rate / 20);
        else
            return 0;
    }
    else if (rate > 0 && rate <= 5){
        return 0;
    }
    else{
        return 0;
    }
}

/**
 * @brief Ballistic::calc_shoot_rate
 * KF的自动射频
 * @param sigma 卡尔曼误差
 */
int Ballistics::calc_shoot_rate(const double & sigma){
    if(sigma < 0.5)
        return 15;
    else if(sigma < 1)
        return 10;
    else if(sigma < 5)
        return (int)(10 - 2.5 * (sigma - 1));
    else
        return 0;//计算射频
}
