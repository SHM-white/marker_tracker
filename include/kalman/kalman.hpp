#ifndef KALMAN_HPP
#define KALMAN_HPP

#define KALMAN_PREDICT_MAX 40
#define DAFUEKF_PREDICT_MAX 50
#define EKF_PREDICT_MAX 300
#define QUEEN_LENGTH 20 //计算方差的窗口，窗口越大，滞后越大

#include <cmath>

#include <iostream>
#include <vector>
#include <atomic>

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <angles/angles.h>

#include <rclcpp/rclcpp.hpp>

#include "params/kalman_params.h"

struct KalmanOutput{
    double x;
    double v_x;
    double a_x;
    double y;
    double v_y;
    double a_y;
    double z;
    double v_z;
    double a_z;
    bool is_success;
    float sigma;
};

struct EkfOutput{
    double x;
    double y;
    double theta;
    double x0;
    double y0;
    double w;
    double R_current;
    double R_next;
    double z_current;
    double z_next;
    bool is_success;
    float sigma;
};

struct DafuOutput{
    double ang;
    double a;
    double w;
    double v;
    double psi;

    bool is_success;
    float sigma;
};

class Kalman {
private:
    uint8_t mode;//使用的卡尔曼模型，目前提供CA和CV模型
    uint8_t missing_cnt;
    uint8_t noise_cnt;
    uint8_t stateSize; //state variable's dimenssion
    uint8_t measSize; //measurement variable's dimession
    uint8_t uSize; //control variables's dimenssion
    double T;//卡尔曼周期
    //计算估计方差
    float numerator;
    float sigma_x[QUEEN_LENGTH] = {0};
    float sigma_y[QUEEN_LENGTH] = {0};
    float sigma_z[QUEEN_LENGTH] = {0};

    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    float sum_x2 = 0;
    float sum_y2 = 0;
    float sum_z2 = 0;

    float sum_ix = 0;
    float sum_iy = 0;
    float sum_iz = 0;
    uint8_t sigma_index;

    Eigen::VectorXd x;  //状态量
    Eigen::VectorXd x_last;  //状态量
    Eigen::VectorXd z;  //先验估计的观测量
    Eigen::VectorXd z_abnomal;//记录到的异常噪音
    Eigen::MatrixXd A;  //状态转移矩阵
    Eigen::MatrixXd B;  //控制矩阵
    Eigen::VectorXd u;  //输入矩阵
    Eigen::MatrixXd P;  //先验估计协方差
    Eigen::MatrixXd H;  //观测矩阵
    Eigen::MatrixXd R;  //测量噪声协方差
    Eigen::MatrixXd Q;  //过程噪声协方差s

public:
    enum KalmanType {
        CVMODE = 0,
        CAMODE = 1,
        CVROTA = 2
    };

    explicit Kalman(uint8_t _mode);

    ~Kalman() = default;

    void init_x(Eigen::VectorXd& x_);

    void Q_set_3d_CV();

    void R_set_3d_CV();

    void Q_set_1d_CV();

    void R_set_1d_CV();

    void Q_set_3d_CA();

    void R_set_3d_CA();

    void P_init(Eigen::MatrixXd& P_);

    void T_set();

    void H_set(Eigen::MatrixXd& H_);

    void A_set(Eigen::MatrixXd& A_);

    void H_set_3d_CV();

    void A_set_3d_CV();

    void H_set_1d_CV();

    void A_set_1d_CV();

    void H_set_3d_CA();

    void A_set_3d_CA();

    float sigma_caculate(const Eigen::Vector3d& _wordCoord);

    float a_Var();


    Eigen::VectorXd predict(Eigen::MatrixXd& A_);

    Eigen::VectorXd predict();

    Eigen::VectorXd predict(Eigen::MatrixXd& A_, Eigen::MatrixXd& B_, Eigen::VectorXd& u_);

    Eigen::VectorXd update(const Eigen::VectorXd& z_meas);

    Eigen::VectorXd update(const Eigen::MatrixXd& H_, const Eigen::VectorXd& z_meas);

    Eigen::VectorXd update(const Eigen::VectorXd& z_meas, const Eigen::MatrixXd& _rota_matrix);

    KalmanOutput
    kalman_filter(bool is_unsucceeseful, const Eigen::VectorXd& _wordCoord, const Eigen::MatrixXd& _rota_matrix);

    KalmanOutput kalman_filter(bool is_unsucceeseful, const Eigen::VectorXd& _wordCoord);

    bool is_large_noise(const Eigen::VectorXd& _currnet_wordCoord);
};

class ExternKalman {
private:
    bool is_R_big{true};
    bool init_value_flag;
    double first_start_time = 0;
    double second_start_time = 0;

    int missing_cnt;
    uint8_t noise_cnt;
    uint8_t stateSize; //state variable's dimenssion
    uint8_t measSize; //measurement variable's dimession
    uint8_t uSize; //control variables's dimenssion
    double R_big, R_small, H_big, H_small;

    float numerator;
    float sigma_w[QUEEN_LENGTH] = {0};
    float sum_w = 0;
    float sum_w2 = 0;
    float sum_iw = 0;
    double yaw = 0;


    uint8_t sigma_index;

    double T;//卡尔曼周期
    //计算线性回归方差,仅限CV模型
    Eigen::VectorXd x;  //状态量
    Eigen::VectorXd x_init;//用于初始化状态量
    Eigen::VectorXd x_last;  //状态量
    Eigen::VectorXd z;  //先验估计的观测量
    Eigen::VectorXd z_last;//上一次观测值，用于判断装甲板跳动
    Eigen::MatrixXd F;  //状态转移矩阵
    Eigen::MatrixXd B;  //控制矩阵
    Eigen::VectorXd u;  //输入矩阵
    Eigen::MatrixXd P;  //先验估计协方差
    Eigen::MatrixXd H;  //观测矩阵
    Eigen::MatrixXd R;  //测量噪声协方差
    Eigen::MatrixXd Q;  //过程噪声协方差s
    //

public:
    explicit ExternKalman();

    ~ExternKalman() = default;

    void set_T(double _T);

    Eigen::VectorXd function_f();

    void update_F();

    void set_H();

    void set_Q();

    void set_R();

    Eigen::VectorXd predict();

    Eigen::VectorXd update(const Eigen::VectorXd& z_meas);

    EkfOutput
    extern_kalman_filter(const bool& is_unsucceeseful, Eigen::VectorXd _wordCoord, const int _nubmer_of_board);

    bool change_detect(const Eigen::VectorXd& z_meas);

    float w_Var();

    void reset_state();

    bool detect_exception_capture(const double& armor_yaw, int queue_length);

};

class StationKalman {
private:
    int missing_cnt;
    uint8_t noise_cnt;
    uint8_t stateSize; //state variable's dimenssion
    uint8_t measSize; //measurement variable's dimession
    uint8_t uSize; //control variables's dimenssion
    std::atomic<double> omiga{0.4 * M_PI * 2};

    float sigma_w[QUEEN_LENGTH] = {0};
    float sum_w = 0;
    float sum_w2 = 0;

    uint8_t sigma_index;

    double T;//卡尔曼周期
    //计算线性回归方差,仅限CV模型
    Eigen::VectorXd x;  //状态量
    Eigen::VectorXd x_init;//用于初始化状态量
    Eigen::VectorXd x_last;  //状态量
    Eigen::VectorXd z;  //先验估计的观测量
    Eigen::VectorXd z_last;//上一次观测值，用于判断装甲板跳动
    Eigen::MatrixXd F;  //状态转移矩阵
    Eigen::MatrixXd B;  //控制矩阵
    Eigen::VectorXd u;  //输入矩阵
    Eigen::MatrixXd P;  //先验估计协方差
    Eigen::MatrixXd H;  //观测矩阵
    Eigen::MatrixXd R;  //测量噪声协方差
    Eigen::MatrixXd Q;  //过程噪声协方差s
    //

public:
    explicit StationKalman();

    ~StationKalman() = default;

    void set_T(double _T);

    Eigen::VectorXd function_f();

    void set_omiga(bool _is_fast, bool _direction);

    void update_F();

    void set_H();

    void set_Q();

    void set_R();

    Eigen::VectorXd predict();

    Eigen::VectorXd update(const Eigen::VectorXd& z_meas);

    EkfOutput extern_kalman_filter(const bool is_unsucceeseful, Eigen::VectorXd& _wordCoord);

    bool change_detect(const Eigen::VectorXd& z_meas);

    float w_Var();

    void reset_state();

};

class DafuKalman {
private:

    int init_flag;
    bool direction;
    double init_start_time;
    double first_angle;

    uint8_t missing_cnt;
    uint8_t noise_cnt;
    uint8_t stateSize; //state variable's dimenssion
    uint8_t measSize; //measurement variable's dimession
    uint8_t uSize; //control variables's dimenssion

    uint8_t sigma_index;

    double T;//卡尔曼周期
    //计算线性回归方差,仅限CV模型
    Eigen::VectorXd x;  //状态量
    Eigen::VectorXd x_init;  //用于初始化状态量
    Eigen::VectorXd x_last;  //状态量
    Eigen::VectorXd z;  //先验估计的观测量
    Eigen::VectorXd z_abnomal;
    Eigen::MatrixXd F;  //状态转移矩阵
    Eigen::MatrixXd B;  //控制矩阵
    Eigen::VectorXd u;  //输入矩阵
    Eigen::MatrixXd P;  //先验估计协方差
    Eigen::MatrixXd H;  //观测矩阵
    Eigen::MatrixXd R;  //测量噪声协方差
    Eigen::MatrixXd Q;  //过程噪声协方差s
    //

public:
    explicit DafuKalman();

    ~DafuKalman() = default;

    void set_T(double _T);

    Eigen::VectorXd function_f();

    void reset_state();

    void update_F();

    void set_H();

    void set_Q();

    void set_R();

    Eigen::VectorXd predict();

    Eigen::VectorXd update(const Eigen::VectorXd& z_meas);

    DafuOutput extern_kalman_filter(bool is_unsucceeseful, Eigen::VectorXd& _wordCoord);

    bool is_large_noise(const Eigen::VectorXd& _currnet_wordCoord);

    bool is_direction_positive();

};

int
IMM_kalman(const bool& is_unsuccessful, const Eigen::VectorXd& _wordCoord, double& _yaw, const int _number_of_board,
           Kalman* _kalman_CV, ExternKalman* _ekf_ROTA, KalmanOutput& _kf_output, EkfOutput& _ekf_output,
           const Eigen::Matrix3d& _rota_matrix3);

template<typename T>
T Limit(T _in, T _limit) {
    T _out = (_in > _limit) ? _limit : _in;
    return (_out < -_limit) ? -_limit : _out;
}

#endif
