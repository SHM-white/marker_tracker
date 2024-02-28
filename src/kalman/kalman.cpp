#include "kalman/kalman.hpp"

inline double sign(double t) {
    if (t > 0) {
        return 1;
    } else {
        return -1;
    }
}

// sateSize状态量个数
// uSize输入的维度
using namespace std;

Kalman::Kalman(uint8_t _mode) : mode(_mode) {
    sigma_index = 0;
    noise_cnt = 0;
    missing_cnt = KALMAN_PREDICT_MAX;
    switch (mode) {
        case KalmanType::CVMODE:
            stateSize = 6;
            measSize = 3;
            uSize = 0;
            break;
        case KalmanType::CAMODE:
            stateSize = 9;
            measSize = 3;
            uSize = 0;
            break;
        case KalmanType::CVROTA:
            stateSize = 2;
            measSize = 1;
            uSize = 0;
            break;
    }
    if (stateSize == 0 || measSize == 0) {
        std::cerr << "Error, State size and measurement size must bigger than 0\n";
    }
    x.resize(stateSize);
    x.setOnes();

    x_last.resize(stateSize);
    x_last.setOnes();

    A.resize(stateSize, stateSize);
    A.setIdentity();

    u.resize(uSize);
    u.transpose();
    u.setZero();

    B.resize(stateSize, uSize);
    B.setZero();

    P.resize(stateSize, stateSize);
    P.setIdentity();

    H.resize(measSize, stateSize);
    H.setZero();

    z.resize(measSize);
    z.setZero();

    z_abnomal.resize(measSize);
    z_abnomal.setZero();

    Q.resize(stateSize, stateSize);
    Q.setZero();

    R.resize(measSize, measSize);
    R.setZero();

    T_set();
    switch (mode) {
        case KalmanType::CVMODE:
            A_set_3d_CV();//设置CV模型的状态矩阵
            H_set_3d_CV();//设置CV模型的测量矩阵
            Q_set_3d_CV();//设置CV模型的状态噪音
            R_set_3d_CV();//设置CV模型的测量噪音
            for (int i = 1; i < QUEEN_LENGTH + 1; i++) {
                numerator += i * i;
            }
            numerator = numerator - QUEEN_LENGTH * (QUEEN_LENGTH + 1) * (QUEEN_LENGTH + 1) * 0.25f;
            break;
        case KalmanType::CAMODE:
            A_set_3d_CA();//设置CV模型的状态矩阵
            H_set_3d_CA();//设置CV模型的测量矩阵
            Q_set_3d_CA();//设置CV模型的状态噪音
            R_set_3d_CA();//设置CV模型的测量噪音
            break;
        case KalmanType::CVROTA:
            A_set_1d_CV();//设置CV模型的状态矩阵
            H_set_1d_CV();//设置CV模型的测量矩阵
            Q_set_1d_CV();//设置CV模型的状态噪音
            R_set_1d_CV();//设置CV模型的测量噪音
            break;

    }
}

void Kalman::P_init(Eigen::MatrixXd& P_) {
    P = P_;
}

//初始化状态噪音
void Kalman::Q_set_3d_CV() {
    double Q_ax, Q_ay, Q_az;
    Q_ax = kalManParams.get__Qx_gain();
    Q_ay = kalManParams.get__Qx_gain();
    Q_az = kalManParams.get__Qx_gain();

    Q << 0.25 * pow(T, 4) * Q_ax, 0.5 * pow(T, 3) * Q_ax, 0, 0, 0, 0,
            0.5 * pow(T, 3) * Q_ax, pow(T, 2) * Q_ax, 0, 0, 0, 0,
            0, 0, 0.25 * pow(T, 4) * Q_ay, 0.5 * pow(T, 3) * Q_ay, 0, 0,
            0, 0, 0.5 * pow(T, 3) * Q_ay, pow(T, 2) * Q_ay, 0, 0,
            0, 0, 0, 0, 0.25 * pow(T, 4) * Q_az, 0.5 * pow(T, 3) * Q_az,
            0, 0, 0, 0, 0.5 * pow(T, 3) * Q_az, pow(T, 2) * Q_az;
}

void Kalman::Q_set_1d_CV() {
    double Q_ang;
    Q_ang = kalManParams.get__Qa_gain();
    Q << 0.25 * pow(T, 4) * Q_ang, 0.5 * pow(T, 3) * Q_ang,
            0.5 * pow(T, 3) * Q_ang, pow(T, 2) * Q_ang;
}

void Kalman::Q_set_3d_CA() {
    double Q_ax, Q_ay, Q_az;
    Q_ax = kalManParams.get__Qx_gain();
    Q_ay = kalManParams.get__Qx_gain();
    Q_az = kalManParams.get__Qx_gain();

    Eigen::MatrixXd U;
    U.resize(measSize, measSize);
    U << Q_ax, 0, 0,
            0, Q_ay, 0,
            0, 0, Q_az;
    Eigen::MatrixXd G;
    G.resize(stateSize, measSize);
    G << pow(T, 3) / 6.0f, 0, 0,
            0.5f * pow(T, 2), 0, 0,
            T, 0, 0,
            0, pow(T, 3) / 6.0f, 0,
            0, 0.5f * pow(T, 2), 0,
            0, T, 0,
            0, 0, pow(T, 3) / 6.0f,
            0, 0, 0.5f * pow(T, 2),
            0, 0, T;
    Q = G * U * G.transpose();
}

//初始化测量噪音
void Kalman::R_set_3d_CV() {
    double R_ax, R_ay, R_az;
    R_ax = kalManParams.get__Rx_gain();
    R_ay = kalManParams.get__Rx_gain();
    R_az = kalManParams.get__Rx_gain();
    R << R_ax * 3, 0, 0,
            0, R_ay, 0,
            0, 0, R_az;
}

void Kalman::R_set_1d_CV() {
    double R_ang;
    R_ang = kalManParams.get__Ra_gain();
    R << R_ang;
}

void Kalman::R_set_3d_CA() {
    double R_ax, R_ay, R_az;
    int cam_idx;
    R_ax = kalManParams.get__Rx_gain();
    R_ay = kalManParams.get__Rx_gain();
    R_az = kalManParams.get__Rx_gain();
    R << R_ax * 3, 0, 0,
            0, R_ay, 0,
            0, 0, R_az;
}

//初始化，卡尔曼滤波初始化只需要初始化初始状态，初始协方差矩阵
void Kalman::init_x(Eigen::VectorXd& x_) {
    x = x_;
}

void Kalman::H_set(Eigen::MatrixXd& H_) {
    H = H_;
}

void Kalman::A_set(Eigen::MatrixXd& A_) {
    A = A_;
}

void Kalman::H_set_3d_CV() {
    H << 1, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 1, 0; //设置测量矩阵
}

void Kalman::H_set_1d_CV() {
    H << 1, 0; //设置测量矩阵
}

void Kalman::H_set_3d_CA() {
    H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0; //设置测量矩阵
}

void Kalman::A_set_3d_CV() {
    A << 1, T, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, T, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, T,
            0, 0, 0, 0, 0, 1; //设置状态矩阵

}

void Kalman::A_set_1d_CV() {
    A << 1, T,
            0, 1; //设置状态矩阵
}

void Kalman::A_set_3d_CA() {
    A << 1, T, 0.5 * pow(T, 2), 0, 0, 0, 0, 0, 0,
            0, 1, T, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, T, 0.5 * pow(T, 2), 0, 0, 0,
            0, 0, 0, 0, 1, T, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, T, 0.5 * pow(T, 2),
            0, 0, 0, 0, 0, 0, 0, 1, T,
            0, 0, 0, 0, 0, 0, 0, 0, 1;
}

void Kalman::T_set() {
    T = kalManParams.get__Dt();
}

float Kalman::sigma_caculate(const Eigen::Vector3d& _wordCoord) {
    sigma_index = sigma_index % QUEEN_LENGTH;
    sum_ix += QUEEN_LENGTH * _wordCoord(0) - sum_x;
    sum_x += _wordCoord(0) - sigma_x[sigma_index];
    sum_iy += QUEEN_LENGTH * _wordCoord(1) - sum_y;
    sum_y += _wordCoord(1) - sigma_y[sigma_index];
    sum_iz += QUEEN_LENGTH * _wordCoord(2) - sum_z;
    sum_z += _wordCoord(2) - sigma_z[sigma_index];
    sigma_x[sigma_index] = _wordCoord(0);
    sigma_y[sigma_index] = _wordCoord(1);
    sigma_z[sigma_index] = _wordCoord(2);
    float b_x = (sum_ix - sum_x * (QUEEN_LENGTH + 1) * 0.5f) / numerator;
    float a_x = sum_x / ((float) QUEEN_LENGTH) - b_x * (QUEEN_LENGTH + 1) * 0.5f;
    float mean_x = 0;
    float b_y = (sum_iy - sum_y * (QUEEN_LENGTH + 1) * 0.5f) / numerator;
    float a_y = sum_y / ((float) QUEEN_LENGTH) - b_y * (QUEEN_LENGTH + 1) * 0.5f;
    float mean_y = 0;
    float b_z = (sum_iz - sum_z * (QUEEN_LENGTH + 1) * 0.5f) / numerator;
    float a_z = sum_z / ((float) QUEEN_LENGTH) - b_z * (QUEEN_LENGTH + 1) * 0.5f;
    float mean_z = 0;
    for (int i = 0; i < QUEEN_LENGTH; ++i) {
        mean_x += powf(sigma_x[(i + sigma_index + 1) % QUEEN_LENGTH] - (b_x * i + a_x), 2);
        mean_y += powf(sigma_y[(i + sigma_index + 1) % QUEEN_LENGTH] - (b_y * i + a_y), 2);
        mean_z += powf(sigma_z[(i + sigma_index + 1) % QUEEN_LENGTH] - (b_z * i + a_z), 2);
    }
    mean_x = sqrt(mean_x / (float) QUEEN_LENGTH);
    mean_y = sqrt(mean_y / (float) QUEEN_LENGTH);
    mean_z = sqrt(mean_z / (float) QUEEN_LENGTH);
    sigma_index++;
    float max_mean = mean_x;
    if (mean_y > max_mean) {
        max_mean = mean_y;
    } else if (mean_z > max_mean) {
        max_mean = mean_z;
    }
    return max_mean;
}

float Kalman::a_Var() {
    sigma_index = sigma_index % QUEEN_LENGTH;
    sum_x2 += pow(x(2), 2) - pow(sigma_x[sigma_index], 2);
    sum_x += x(2) - sigma_x[sigma_index];
    sum_y2 += pow(x(5), 2) - pow(sigma_y[sigma_index], 2);
    sum_y += x(5) - sigma_y[sigma_index];
    sum_z2 += pow(x(8), 2) - pow(sigma_z[sigma_index], 2);
    sum_z += x(8) - sigma_z[sigma_index];
    sigma_x[sigma_index] = x(2);
    sigma_y[sigma_index] = x(5);
    sigma_z[sigma_index] = x(8);
    sigma_index++;
    float x_k2 = sum_x2 - pow(sum_x, 2) / (float) QUEEN_LENGTH;
    float y_k2 = sum_y2 - pow(sum_y, 2) / (float) QUEEN_LENGTH;
    float z_k2 = sum_z2 - pow(sum_z, 2) / (float) QUEEN_LENGTH;
    x_k2 = sqrt(x_k2 / (float) QUEEN_LENGTH);
    y_k2 = sqrt(y_k2 / (float) QUEEN_LENGTH);
    z_k2 = sqrt(z_k2 / (float) QUEEN_LENGTH);
    float max_k2 = x_k2;
    if (y_k2 > max_k2)
        max_k2 = y_k2;
    if (z_k2 > max_k2)
        max_k2 = z_k2;
    return max_k2;
}

//有输入矩阵和控制矩阵的卡尔曼滤波预测过程
Eigen::VectorXd Kalman::predict(Eigen::MatrixXd& A_, Eigen::MatrixXd& B_, Eigen::VectorXd& u_) {
    A = A_;
    B = B_;
    u = u_;
    x = A * x + B * u;//根据时刻t-1的状态由状态转换举着预测时刻t的先验估计值，
    Eigen::VectorXd A_T = A.transpose();
    P = A * P * A_T + Q;//时刻t先验估计值的的协方差,Q为过程噪声的协方差
    return x;
}

//没有输入矩阵和控制矩阵的卡尔曼滤波预测过程
Eigen::VectorXd Kalman::predict(Eigen::MatrixXd& A_) {
    A = A_;
    x = A * x;
    Eigen::MatrixXd A_T = A.transpose();
    P = A * P * A_T + Q;
    //  cout << "P-=" << P<< endl;
    return x;
}

//预测模型不变
Eigen::VectorXd Kalman::predict() {
    x_last = x;
    x = A * x;
    Eigen::MatrixXd A_T = A.transpose();
    P = A * P * A_T + Q;
    //  cout << "P-=" << P<< endl;
    return x;
}

// H_   观测矩阵
//z_means 观测量，由系统真实观测输入
Eigen::VectorXd Kalman::update(const Eigen::MatrixXd& H_, const Eigen::VectorXd& z_meas) {
    H = H_;
    Eigen::MatrixXd temp1, temp2, Ht;
    Ht = H.transpose();
    temp1 = H * P * Ht + R;
    temp2 = temp1.inverse();//(H*P*H'+R)^(-1)
    Eigen::MatrixXd K = P * Ht * temp2;
    z = H * x;
    x = x + K * (z_meas - z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);//Identity()单位阵
    P = (I - K * H) * P;
    //  cout << "P=" << P << endl;
    return x;
}

//测量模型不变
Eigen::VectorXd Kalman::update(const Eigen::VectorXd& z_meas) {
    Eigen::MatrixXd temp1, temp2, Ht;
    Ht = H.transpose();
    temp1 = H * P * Ht + R;
    temp2 = temp1.inverse();//(H*P*H'+R)^(-1)
    Eigen::MatrixXd K = P * Ht * temp2;
    z = H * x;
    x = x + K * (z_meas - z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);//Identity()单位阵
    P = (I - K * H) * P;
    //  cout << "P=" << P << endl;
    return x;
}

//测量模型不变,噪音变化
Eigen::VectorXd Kalman::update(const Eigen::VectorXd& z_meas, const Eigen::MatrixXd& rota_matrix) {
    Eigen::MatrixXd temp1, temp2, Ht;
    Ht = H.transpose();
    R = rota_matrix * R * rota_matrix.transpose();
    temp1 = H * P * Ht + R;
    temp2 = temp1.inverse();//(H*P*H'+R)^(-1)
    Eigen::MatrixXd K = P * Ht * temp2;
    z = H * x;
    x = x + K * (z_meas - z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);//Identity()单位阵
    P = (I - K * H) * P;
    //  cout << "P=" << P << endl;
    return x;
}

KalmanOutput
Kalman::kalman_filter(bool is_unsucceeseful, const Eigen::VectorXd& _wordCoord, const Eigen::MatrixXd& _rota_matrix) {
    KalmanOutput output;
    //判断四种情况
    if (missing_cnt >= KALMAN_PREDICT_MAX && !is_unsucceeseful) {
        Eigen::VectorXd x_init;
        x_init.resize(stateSize);
        switch (stateSize) {
            case 2:
                x_init << _wordCoord(0), 0;
                break;
            case 6:
                x_init << _wordCoord(0), 0, _wordCoord(1), 0, _wordCoord(2), 0;
                break;
            case 9:
                x_init << _wordCoord(0), 0, 0, _wordCoord(1), 0, 0, _wordCoord(2), 0, 0;
                break;
        }
        init_x(x_init);
        z_abnomal = _wordCoord;//初始化异常噪音
        missing_cnt = 0;
        output.is_success = false;
    } else if (!is_unsucceeseful) {//识别成功
        predict();
        if (is_large_noise(_wordCoord));//处理异常噪音
        else {
            update(_wordCoord, _rota_matrix);
        }

        missing_cnt = 0;
        output.is_success = true;
        //ROS_INFO("yuce,x:%f,vx:%f,y:%f,vy:%f,z:%f,vz:%f",kalman_x_vector(0),kalman_x_vector(1),kalman_x_vector(2),kalman_x_vector(3),kalman_x_vector(4),kalman_x_vector(5));
    } else if (is_unsucceeseful && (missing_cnt < KALMAN_PREDICT_MAX)) {//不成功但是在KALMAN_PREDICT_MAX步内
        predict();
        missing_cnt++;
        output.is_success = true;
        //ROS_INFO("celiang,x:%f,vx:%f,y:%f,vy:%f,z:%f,vz:%f",kalman_x_vector(0),kalman_x_vector(1),kalman_x_vector(2),kalman_x_vector(3),kalman_x_vector(4),kalman_x_vector(5));
    } else {//丢失时间过长,判定丢失
        missing_cnt = KALMAN_PREDICT_MAX;
        output.is_success = false;
    }

    switch (stateSize) {
        case 6:
            if (sqrt(x(1) * x(1) + x(3) * x(3) + x(5) * x(5)) > 2500) {//估计的速度过大
                missing_cnt = KALMAN_PREDICT_MAX;
                output.is_success = false;
            }
            break;
        case 9:
            if (sqrt(x(1) * x(1) + x(4) * x(4) + x(7) * x(7)) > 2500) {//估计的速度过大
                missing_cnt = KALMAN_PREDICT_MAX;
                output.is_success = false;
            }
            break;
    }


    switch (mode) {
        case KalmanType::CVMODE:
            output.x = x(0);
            output.v_x = x(1);
            output.y = x(2);
            output.v_y = x(3);
            output.z = x(4);
            output.v_z = x(5);
            output.a_x = 0;
            output.a_y = 0;
            output.a_z = 0;
            output.sigma = sigma_caculate(_wordCoord);
            //std::cout << "kalman_CV ---------------------------------" << std::endl;
            break;
        case KalmanType::CAMODE:
            output.x = x(0);
            output.v_x = x(1);
            output.a_x = x(2);
            output.y = x(3);
            output.v_y = x(4);
            output.a_y = x(5);
            output.z = x(6);
            output.v_z = x(7);
            output.a_z = x(8);
            output.sigma = a_Var();

            //std::cout << "kalman_CA ---------------------------------" << std::endl;
            break;
        case KalmanType::CVROTA:
            output.x = x(0);
            output.v_x = x(1);

            //std::cout << "kalman_CVROTE ---------------------------------" << std::endl;
            break;

    }
    return output;
}

KalmanOutput Kalman::kalman_filter(bool is_unsucceeseful, const Eigen::VectorXd& _wordCoord) {
    KalmanOutput output;
    //判断四种情况
    if (missing_cnt >= KALMAN_PREDICT_MAX && !is_unsucceeseful) {
        Eigen::VectorXd x_init;
        x_init.resize(stateSize);
        switch (stateSize) {
            case 2:
                x_init << _wordCoord(0), 0;
                break;
            case 6:
                x_init << _wordCoord(0), 0, _wordCoord(1), 0, _wordCoord(2), 0;
                break;
            case 9:
                x_init << _wordCoord(0), 0, 0, _wordCoord(1), 0, 0, _wordCoord(2), 0, 0;
                break;
        }
        init_x(x_init);
        z_abnomal = _wordCoord;//初始化异常噪音
        missing_cnt = 0;
        output.is_success = false;
    } else if (!is_unsucceeseful) {//识别成功
        predict();
        if (is_large_noise(_wordCoord));//处理异常噪音
        else {
            update(_wordCoord);
        }

        missing_cnt = 0;
        output.is_success = true;
        //ROS_INFO("yuce,x:%f,vx:%f,y:%f,vy:%f,z:%f,vz:%f",kalman_x_vector(0),kalman_x_vector(1),kalman_x_vector(2),kalman_x_vector(3),kalman_x_vector(4),kalman_x_vector(5));
    } else if (is_unsucceeseful && (missing_cnt < KALMAN_PREDICT_MAX)) {//不成功但是在KALMAN_PREDICT_MAX步内
        predict();
        missing_cnt++;
        output.is_success = true;
        //ROS_INFO("celiang,x:%f,vx:%f,y:%f,vy:%f,z:%f,vz:%f",kalman_x_vector(0),kalman_x_vector(1),kalman_x_vector(2),kalman_x_vector(3),kalman_x_vector(4),kalman_x_vector(5));
    } else {//丢失时间过长,判定丢失
        missing_cnt = KALMAN_PREDICT_MAX;
        output.is_success = false;
    }
    switch (stateSize) {
        case 2:
            break;
        case 6:
            if (abs(x(1) > 3000 || x(3) > 3000 || x(5) > 3000)) {//估计的速度过大
                missing_cnt = KALMAN_PREDICT_MAX;
                output.is_success = false;
            }
            break;
        case 9:
            if (abs(x(1) > 3000 || x(4) > 3000 || x(7) > 3000)) {//估计的速度过大
                missing_cnt = KALMAN_PREDICT_MAX;
                output.is_success = false;
            }
            break;
    }


    switch (mode) {
        case KalmanType::CVMODE:
            output.x = x(0);
            output.v_x = x(1);
            output.y = x(2);
            output.v_y = x(3);
            output.z = x(4);
            output.v_z = x(5);
            output.a_x = 0;
            output.a_y = 0;
            output.a_z = 0;
            output.sigma = sigma_caculate(_wordCoord);
            std::cout << "kalman_CV ---------------------------------" << std::endl;
            break;
        case KalmanType::CAMODE:
            output.x = x(0);
            output.v_x = x(1);
            output.a_x = x(2);
            output.y = x(3);
            output.v_y = x(4);
            output.a_y = x(5);
            output.z = x(6);
            output.v_z = x(7);
            output.a_z = x(8);
            output.sigma = a_Var();

            std::cout << "kalman_CA ---------------------------------" << std::endl;
            break;
        case KalmanType::CVROTA:
            output.x = x(0);
            output.v_x = x(1);

            std::cout << "kalman_CVROTE ---------------------------------" << std::endl;
            break;

    }
    return output;
}

bool Kalman::is_large_noise(const Eigen::VectorXd& _currnet_wordCoord) {
    switch (measSize) {
        case 1: {
            float noise_x = abs((_currnet_wordCoord(0) - z_abnomal(0)) / z_abnomal(0));
            if (noise_x > 0.15)
                noise_cnt++;
            else
                noise_cnt = 0;

            if (noise_cnt == 0) {
                z_abnomal = _currnet_wordCoord;
                return false;
            } else if (noise_cnt <= 5)//是噪音
                return true;
            else {
                z_abnomal = _currnet_wordCoord;//是突变
                return false;
            }
        }
        case 3: {
            float noise_x = abs((_currnet_wordCoord(0) - z_abnomal(0)) / z_abnomal(0));
            float noise_y = abs((_currnet_wordCoord(1) - z_abnomal(1)) / z_abnomal(1));
            float noise_z = abs((_currnet_wordCoord(2) - z_abnomal(2)) / z_abnomal(2));

            if (noise_x > 0.15 || noise_y > 0.15 || noise_z > 0.15)
                noise_cnt++;
            else
                noise_cnt = 0;

            if (noise_cnt == 0) {
                z_abnomal = _currnet_wordCoord;
                return false;
            } else if (noise_cnt <= 5)//是噪音
                return true;
            else {
                z_abnomal = _currnet_wordCoord;
                return false;
            }
        }
    }
}
/*********************************************反陀螺**************************************************/
/*                     y
 *                     |
 *                     |
 *                     |
 *                     |
 *                     |
 *                     ---------------------x
 *                     小陀罗坐标轴，和车体坐标系的xy不一样*/

ExternKalman::ExternKalman() {
    init_value_flag = 0;//0表示未初始化，2表示初始化完成

    sigma_index = 0;
    noise_cnt = 0;
    missing_cnt = EKF_PREDICT_MAX;

    stateSize = 8;
    measSize = 4;
    uSize = 0;


    x.resize(stateSize);
    x.setOnes();

    x_last.resize(stateSize);
    x_last.setOnes();

    x_init.resize(stateSize);
    x_init << 1, 1, 0, 4, 0, 3, 0.15, 0.2;//初始值指定

    F.resize(stateSize, stateSize);
    F.setZero();

    u.resize(uSize);
    u.transpose();
    u.setZero();

    B.resize(stateSize, uSize);
    B.setZero();

    P.resize(stateSize, stateSize);
    P.setIdentity();

    H.resize(measSize, stateSize);
    H.setZero();

    z.resize(measSize);
    z.setZero();

    z_last.resize(measSize);
    z_last.setZero();

    Q.resize(stateSize, stateSize);
    Q.setZero();

    R.resize(measSize, measSize);
    R.setZero();

    set_T(0.005);//一定先设置T
    set_H();
    set_R();
    set_Q();
}

Eigen::VectorXd ExternKalman::function_f() {
    Eigen::VectorXd x_k = x;
    x_k(0) = x(3) + x(6) * (cos(x(2)) - sin(x(2)) * x(5) * T);
    x_k(1) = x(4) + x(6) * (sin(x(2)) + cos(x(2)) * x(5) * T);
    x_k(2) = x(2) + x(5) * T;
    x_k(3) = x(3);
    x_k(4) = x(4);
    x_k(5) = x(5);//w=w
    x_k(6) = x(6);//R=R
    x_k(7) = x(7); //z=z
    return x_k;
}

void ExternKalman::update_F() {
    double F02 = -x(6) * (sin(x(2)) + x(5) * T * cos(x(2)));  //-R*(sin(theta) + w*T*cos(theta))
    double F05 = -x(6) * sin(x(2)) * T;                                    //-R*sin(theta)*T
    double F06 = cos(x(2)) - sin((x(2))) * x(5) * T;              //cos(theta) - sin(theta)*w*T
    double F12 = x(6) * (cos(x(2) - x(5) * T * sin(x(2))));   //R*(cos(theta) - w*Tsin(theta))
    double F15 = x(6) * cos(x(2)) * T;                                     //R*cos(theta)*T
    double F16 = sin(x(2)) + cos((x(2))) * x(5) * T;              //sin(theta) + cos(theta)*w*T
    //雅可比矩阵
    /*   x  ,y  ,thta,x0,y0 ,w  ,R ,z        */
    F << 0, 0, F02, 1, 0, F05, F06, 0,//           x
            0, 0, F12, 0, 1, F15, F16, 0,//           y
            0, 0, 1, 0, 0, T, 0, 0,//           theta
            0, 0, 0, 1, 0, 0, 0, 0,//           x0
            0, 0, 0, 0, 1, 0, 0, 0,//           y0
            0, 0, 0, 0, 0, 1, 0, 0,//           w
            0, 0, 0, 0, 0, 0, 1, 0,//           R
            0, 0, 0, 0, 0, 0, 0, 1;//           z
}

void ExternKalman::set_H() {
    H << 1, 0, 0, 0, 0, 0, 0, 0,  //x
            0, 1, 0, 0, 0, 0, 0, 0,  //y
            0, 0, 0, 0, 0, 0, 0, 1,  //z
            0, 0, 1, 0, 0, 0, 0, 0;  //theta


}

void ExternKalman::set_T(double _T) {
    T = _T;
}

void ExternKalman::set_Q() {
    double Q_x, Q_y, Q_theta, Q_x0, Q_y0, Q_w, Q_R, Q_z;
    Q_x = kalManParams.get__Q_x();
    Q_y = kalManParams.get__Q_y();
    Q_theta = kalManParams.get__Q_theta();
    Q_x0 = kalManParams.get__Q_x0();
    Q_y0 = kalManParams.get__Q_y0();
    Q_w = kalManParams.get__Q_w();
    Q_R = kalManParams.get__Q_R();
    Q_z = kalManParams.get__Q_z();

    Eigen::DiagonalMatrix<double, 8> Q_diag;
    Q_diag.diagonal() << Q_x, Q_y, Q_theta, Q_x0, Q_y0, Q_w, Q_R, Q_z;
    Q = Q_diag;
}

void ExternKalman::set_R() {
    double R_x, R_y, R_z, R_theta;
    R_x = kalManParams.get__R_x();
    R_y = kalManParams.get__R_y();
    R_z = kalManParams.get__R_z();
    R_theta = kalManParams.get__R_theta();

    Eigen::DiagonalMatrix<double, 4> R_diag;
    R_diag.diagonal() << R_x, R_y, R_z, R_theta;
    R = R_diag;
}

Eigen::VectorXd ExternKalman::update(const Eigen::VectorXd& z_meas) {
    Eigen::MatrixXd temp1, temp2, Ht;
    Ht = H.transpose();
    temp1 = H * P * Ht + R;
    temp2 = temp1.inverse(); //(H*P*H'+R)^(-1)
    Eigen::MatrixXd K = P * Ht * temp2; // Kk = P*H'*(H*P*H'+R)^(-1)
    z = H * x;
    x = x + K * (z_meas - z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);//Identity()单位阵
    P = (I - K * H) * P;
    //  cout << "P=" << P << endl;
    x(6) = x(6) < 0 ? 0 : x(6);
    return x;
}

Eigen::VectorXd ExternKalman::predict() {
    x_last = x;
    update_F();//计算雅可比矩阵
    x = function_f();
    Eigen::MatrixXd F_T = F.transpose();
    P = F * P * F_T + F * Q * F_T;
    //  cout << "P-=" << P<< endl;
    x(6) = x(6) < 0 ? 0 : x(6);
    return x;
}

EkfOutput ExternKalman::extern_kalman_filter(const bool& is_unsucceeseful, Eigen::VectorXd _wordCoord,
                                             const int _nubmer_of_board) {

    EkfOutput output_temp;
    bool detected = !is_unsucceeseful;
    bool change = change_detect(_wordCoord);


    static double last_yaw = yaw;
    //detected = detect_exception_capture(_wordCoord(3), 5) && detected;
    if (detected) {          //角度连续化
        double diff_angle = angles::shortest_angular_distance(last_yaw, _wordCoord(3));
        yaw += diff_angle;
        last_yaw += diff_angle;
    }

    if (change && detected) {  //通过装甲板跳变计算真实的角速度
        second_start_time = first_start_time;
        first_start_time = rclcpp::Clock().now().seconds();
        if (first_start_time != 0 && second_start_time != 0)
            init_value_flag = true;
        else
            init_value_flag = false;
    }

    if (!init_value_flag) {             //ekf角速度的初始值是否计算完毕
        output_temp.is_success = false;
    } else if (detected && missing_cnt == EKF_PREDICT_MAX) {//给ekf设置初始状态量
        /*   x  ,y  ,thta,x0,y0 ,w  ,R ,z        */
        x_init(0) = _wordCoord(0);
        x_init(1) = _wordCoord(1);
        x_init(7) = _wordCoord(2);
        x_init(2) = yaw;
        x_init(3) = _wordCoord(0) - x_init(6) * cos(yaw);
        x_init(4) = _wordCoord(1) - x_init(6) * sin(yaw);
        x_init(5) = 0;
        x = x_init;                        //初始化状态量，其中前两个用测量位置初始化
        R_big = R_small = x(6);
        H_big = H_small = x(7);

        z_last = _wordCoord;               //设置上一次观测值
        missing_cnt = 0;
        output_temp.is_success = false;
    } else if (detected) {          //识别成功进行预测更新
        if (change) { //判断有没有装甲板跳变,切换状态
            //yaw -= 2 * M_PI / (double)_nubmer_of_board * sign(x(5));
            x(2) = yaw;
            x(0) = _wordCoord(0);
            x(1) = _wordCoord(1);

            if (is_R_big) {//切换高低装甲板，加速收敛
                R_big = x(6);
                H_big = x(7);
                x(6) = R_small;
                x(7) = H_small;
                is_R_big = false;
            } else {
                R_small = x(6);
                H_small = x(7);
                x(6) = R_big;
                x(7) = H_big;
                is_R_big = true;
            }
        } else;
        _wordCoord(3) = yaw;
        predict();//预测
        x_init = update(_wordCoord);//不断更新初始状态，为下一次丢失作准备
        missing_cnt = 0;
        output_temp.is_success = true;

    } else if (!detected && missing_cnt < EKF_PREDICT_MAX) {//没识别到但是在KALMAN_PREDICT_MAX步内
        predict();
        missing_cnt++;
        output_temp.is_success = true;

    } else {//丢失时间过长,判定丢失
        missing_cnt = EKF_PREDICT_MAX;              //设置追踪丢失
        output_temp.is_success = false;
        first_start_time = second_start_time = 0;   //重新设置时间点
    }

    output_temp.x = -x(0);
    output_temp.y = -x(1);
    output_temp.theta = x(2) - M_PI;
    output_temp.x0 = -x(3);
    output_temp.y0 = -x(4);
    output_temp.w = x(5);
    output_temp.R_current = x(6);
    output_temp.z_current = x(7);
    if (is_R_big) {
        output_temp.R_next = R_small;
        output_temp.z_next = H_small;
    } else {
        output_temp.R_next = R_big;
        output_temp.z_next = H_big;
    }
    output_temp.sigma = w_Var();
    return output_temp;
}


bool ExternKalman::change_detect(const Eigen::VectorXd& z_meas) {
    float error_x = z_meas(0) - z_last(0);
    float error_y = z_meas(1) - z_last(1);
    float error_yaw = angles::shortest_angular_distance(z_meas(3), z_last(3));

    z_last = z_meas;
    //TODO:static ros::Publisher change_details = mh.advertise<std_msgs::Float32>("/change_details_show", 1);
//    static std_msgs::Float32 change_details_msg;
//    change_details_msg.data = sqrt(pow(error_x, 2) + pow(error_y, 2)) + pow(error_yaw, 2) / 2;
//    change_details.publish(change_details_msg);
    if (sqrt(pow(error_x, 2) + pow(error_y, 2)) + pow(error_yaw, 2) / 2 > 0.26) {//通过两次之间的距离判断跳变
        return true;
    } else {
        return false;
    }
}

float ExternKalman::w_Var() {
    sigma_index = sigma_index % QUEEN_LENGTH;
    sum_w2 += pow(x(5), 2) - pow(sigma_w[sigma_index], 2);
    sum_w += x(5) - sigma_w[sigma_index];
    sigma_w[sigma_index] = x(5);
    float w_k2 = sum_w2 - pow(sum_w, 2) / (float) QUEEN_LENGTH;
    w_k2 = sqrt(w_k2 / (float) QUEEN_LENGTH);
    sigma_index++;
    return w_k2;
}

bool ExternKalman::detect_exception_capture(const double& armor_yaw, int queue_length) {
    static vector<double> yaw_queue;
    if (yaw_queue.size() < queue_length) {
        yaw_queue.push_back(armor_yaw);
    } else {
        yaw_queue.erase(yaw_queue.begin() + 0);
        yaw_queue.push_back(armor_yaw);
    }
    if (abs(angles::shortest_angular_distance(yaw_queue[0], yaw_queue[queue_length - 1])) > M_PI_2 / 2) {
        return false;
    } else {
        return true;
    }

}

void ExternKalman::reset_state() {
    init_value_flag = 0;
    missing_cnt = EKF_PREDICT_MAX;
}

/*********************************************前哨战**************************************************/
StationKalman::StationKalman() {

    sigma_index = 0;
    noise_cnt = 0;
    missing_cnt = EKF_PREDICT_MAX;

    stateSize = 6;
    measSize = 4;
    uSize = 0;


    x.resize(stateSize);
    x.setOnes();

    x_last.resize(stateSize);
    x_last.setOnes();

    x_init.resize(stateSize);
    x_init << 1, 1, 0, 4, 0, 1.5;//初始值指定

    F.resize(stateSize, stateSize);
    F.setZero();

    u.resize(uSize);
    u.transpose();
    u.setZero();

    B.resize(stateSize, uSize);
    B.setZero();

    P.resize(stateSize, stateSize);
    P.setIdentity();

    H.resize(measSize, stateSize);
    H.setZero();

    z.resize(measSize);
    z.setZero();

    z_last.resize(measSize);
    z_last.setZero();

    Q.resize(stateSize, stateSize);
    Q.setZero();

    R.resize(measSize, measSize);
    R.setZero();

    set_T(0.005);//一定先设置T
    set_H();
    set_R();
    set_Q();

}

Eigen::VectorXd StationKalman::function_f() {
    Eigen::VectorXd x_k = x;
    x_k(0) = x(3) + 0.2765 * cos(x(2) + omiga * T);//x
    x_k(1) = x(4) + 0.2765 * sin(x(2) + omiga * T);//y
    x_k(2) = x(2) + omiga * T; //thta
    x_k(3) = x(3);//x0
    x_k(4) = x(4);//y0
    x_k(5) = x(5); //z=z
    return x_k;
}

void StationKalman::update_F() {
    double F02 = -0.2765 * sin(x(2) + omiga * T);  //-R*(sin(theta) + w*T*cos(theta))
    double F12 = 0.2765 * cos(x(2) + omiga * T);   //R*(cos(theta) - w*Tsin(theta))
    //雅可比矩阵
    /*      x  ,y  ,thta,x0,y0,z        */
    F << 0, 0, F02, 1, 0, 0,//           x
            0, 0, F12, 0, 1, 0,//           y
            0, 0, 1, 0, 0, 0,//           theta
            0, 0, 0, 1, 0, 0,//           x0
            0, 0, 0, 0, 1, 0,//           y0
            0, 0, 0, 0, 0, 1;//           z
}

void StationKalman::set_H() {
    H << 1, 0, 0, 0, 0, 0,  //x
            0, 1, 0, 0, 0, 0,  //y
            0, 0, 0, 0, 0, 1,  //z
            0, 0, 1, 0, 0, 0;  //theta


}

void StationKalman::set_T(double _T) {
    T = _T;
}

void StationKalman::set_Q() {
    double Q_x, Q_y, Q_theta, Q_x0, Q_y0, Q_z;
    Q_x = kalManParams.get__Q_x();
    Q_y = kalManParams.get__Q_y();
    Q_theta = kalManParams.get__Q_theta();
    Q_x0 = kalManParams.get__Q_x0();
    Q_y0 = kalManParams.get__Q_y0();
    Q_z = kalManParams.get__Q_z();

    Eigen::DiagonalMatrix<double, 6> Q_diag;
    Q_diag.diagonal() << Q_x, Q_y, Q_theta, Q_x0, Q_y0, Q_z;
    Q = Q_diag;
}

void StationKalman::set_R() {
    double R_x, R_y, R_z, R_theta;
    R_x = kalManParams.get__R_x();
    R_y = kalManParams.get__R_y();
    R_z = kalManParams.get__R_z();
    R_theta = kalManParams.get__R_theta();

    Eigen::DiagonalMatrix<double, 4> R_diag;
    R_diag.diagonal() << R_x, R_y, R_z, R_theta;
    R = R_diag;
}

Eigen::VectorXd StationKalman::update(const Eigen::VectorXd& z_meas) {
    Eigen::MatrixXd temp1, temp2, Ht;
    Ht = H.transpose();
    temp1 = H * P * Ht + R;
    temp2 = temp1.inverse(); //(H*P*H'+R)^(-1)
    Eigen::MatrixXd K = P * Ht * temp2; // Kk = P*H'*(H*P*H'+R)^(-1)
    z = H * x;
    x = x + K * (z_meas - z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);//Identity()单位阵
    P = (I - K * H) * P;
    //  cout << "P=" << P << endl;
    return x;
}

Eigen::VectorXd StationKalman::predict() {
    x_last = x;
    update_F();//计算雅可比矩阵
    x = function_f();
    Eigen::MatrixXd F_T = F.transpose();
    P = F * P * F_T + F * Q * F_T;
    //  cout << "P-=" << P<< endl;
    return x;
}

EkfOutput StationKalman::extern_kalman_filter(const bool is_unsucceeseful, Eigen::VectorXd& _wordCoord) {
    EkfOutput output_temp;
    if (missing_cnt >= EKF_PREDICT_MAX && !is_unsucceeseful) {//用来给ekf设置初始状态量的前两个值

        x_init(0) = _wordCoord(0);
        x_init(1) = _wordCoord(1);
        x_init(5) = _wordCoord(2);
        x_init(2) = _wordCoord(3);
        x_init(3) = x_init(0);
        x_init(4) = x_init(1);
        x = x_init;                        //初始化状态量，其中前两个用测量位置初始化

        z_last = _wordCoord;               //设置上一次观测值
        missing_cnt = 0;
        output_temp.is_success = false;
    } else if (!is_unsucceeseful) {          //识别成功进行预测更新
        if (change_detect(_wordCoord)) { //判断有没有装甲板跳变,切换状态
            x(2) = _wordCoord(3);
            x(0) = _wordCoord(0);
            x(1) = _wordCoord(1);

        } else;

        predict();//预测
        x_init = update(_wordCoord);//不断更新初始状态，为下一次丢失作准备
        missing_cnt = 0;
        output_temp.is_success = true;
        //ROS_INFO("yuce,x:%f,vx:%f,y:%f,vy:%f,z:%f,vz:%f",kalman_x_vector(0),kalman_x_vector(1),kalman_x_vector(2),kalman_x_vector(3),kalman_x_vector(4),kalman_x_vector(5));
    } else if (is_unsucceeseful && (missing_cnt < EKF_PREDICT_MAX)) {//没识别到但是在KALMAN_PREDICT_MAX步内
        predict();
        missing_cnt++;
        output_temp.is_success = true;
        //ROS_INFO("celiang,x:%f,vx:%f,y:%f,vy:%f,z:%f,vz:%f",kalman_x_vector(0),kalman_x_vector(1),kalman_x_vector(2),kalman_x_vector(3),kalman_x_vector(4),kalman_x_vector(5));
    } else {//丢失时间过长,判定丢失
        missing_cnt = EKF_PREDICT_MAX;
        output_temp.is_success = false;
    }
    output_temp.x = -x(0);
    output_temp.y = -x(1);
    output_temp.theta = x(2) - M_PI;
    output_temp.w = omiga;
    output_temp.x0 = -x(3);
    output_temp.y0 = -x(4);
    output_temp.z_current = x(5);

    output_temp.sigma = w_Var();
    return output_temp;
}


bool StationKalman::change_detect(const Eigen::VectorXd& z_meas) {
    float error_x = z_meas(0) - z_last(0);
    float error_y = z_meas(1) - z_last(1);
    float error_yaw = z_meas(3) - z_last(3);

    z_last = z_meas;
    if (sqrt(pow(error_x, 2) + pow(error_y, 2)) > 0.2 || abs(error_yaw) > 0.6) {//通过两次之间的距离判断跳变
        return true;
    } else {
        return false;
    }
}

float StationKalman::w_Var() {
    sigma_index = sigma_index % QUEEN_LENGTH;
    sum_w2 += pow(x(3), 2) - pow(sigma_w[sigma_index], 2);
    sum_w += x(3) - sigma_w[sigma_index];
    sigma_w[sigma_index] = x(3);
    float w_k2 = sum_w2 - pow(sum_w, 2) / (float) QUEEN_LENGTH;
    w_k2 = sqrt(w_k2 / (float) QUEEN_LENGTH);
    sigma_index++;
    return w_k2;
}

void StationKalman::set_omiga(bool _is_fast, bool _direction) {
    if (_is_fast)
        if (_direction)
            omiga = 0.4 * M_PI * 2;
        else
            omiga = -0.4 * M_PI * 2;
    else if (_direction)
        omiga = 0.2 * M_PI * 2;
    else
        omiga = -0.2 * M_PI * 2;

}

void StationKalman::reset_state() {
    missing_cnt = EKF_PREDICT_MAX;
}

/*********************************************大符**************************************************/
DafuKalman::DafuKalman() {
    init_flag = 0;
    init_start_time = 0;

    sigma_index = 0;
    noise_cnt = 0;
    missing_cnt = DAFUEKF_PREDICT_MAX;

    stateSize = 5;
    measSize = 1;
    uSize = 0;


    x.resize(stateSize);
    x.setOnes();

    x_last.resize(stateSize);
    x_last.setOnes();

    x_init.resize(stateSize);
    x_init << 1, 0, 0.0, 0.85, 0.94;//初始值指定

    F.resize(stateSize, stateSize);
    F.setZero();

    u.resize(uSize);
    u.transpose();
    u.setZero();

    B.resize(stateSize, uSize);
    B.setZero();

    P.resize(stateSize, stateSize);
    P.setIdentity();

    H.resize(measSize, stateSize);
    H.setZero();

    z.resize(measSize);
    z.setZero();

    z_abnomal.resize(measSize);
    z_abnomal.setZero();

    Q.resize(stateSize, stateSize);
    Q.setZero();

    R.resize(measSize, measSize);
    R.setZero();

    set_T(0.005);//一定先设置T
    set_H();
    set_R();
    set_Q();

}

Eigen::VectorXd DafuKalman::function_f() {
    Eigen::VectorXd x_k = x;
    if (direction)
        x_k(0) = x(0) + x(1) * T; //ang
    else
        x_k(0) = x(0) - x(1) * T; //ang

    x_k(1) = x(3) * sin(x(2) + x(4) * T) + 2.09 - x(3); //v
    x_k(2) = x(2) + x(4) * T; //theta
    x_k(3) = x(3); //a
    x_k(4) = x(4); //w
    return x_k;
}

void DafuKalman::set_T(double _T) {
    T = _T;
}

void DafuKalman::set_H() {
    H << 1, 0, 0, 0, 0;
}

void DafuKalman::update_F() {
    double F01;
    if (direction)
        F01 = T;
    else
        F01 = -T;

    double F12 = x(3) * cos(x(2) + x(4) * T);
    double F13 = sin(x(2) + x(4) * T);
    double F14 = x(3) * cos(x(2) + x(4) * T) * T;
    //雅可比矩阵
    /*   ang,v  ,theta,a  ,w        */
    F << 1, F01, 0, 0, 0,//        ang
            0, 0, F12, F13, F14,//        v
            0, 0, 1, 0, T,//        theta
            0, 0, 0, 1, 0,//        a
            0, 0, 0, 0, 1;//        w
}

void DafuKalman::set_Q() {
    double Q_ang, Q_v, Q_theta, Q_a, Q_w;
    Q_ang = kalManParams.get__Qa_ang();
    Q_a = kalManParams.get__Qa_a();
    Q_w = kalManParams.get__Qa_w();
    Q_theta = kalManParams.get__Qa_theta();
    Q_v = kalManParams.get__Qa_v();

    Eigen::DiagonalMatrix<double, 5> Q_diag;
    Q_diag.diagonal() << Q_ang, Q_v, Q_theta, Q_a, Q_w;
    Q = Q_diag;
}

void DafuKalman::set_R() {
    double R_ang;
    R_ang = kalManParams.get__Ra_ang();
    R << R_ang;
}

Eigen::VectorXd DafuKalman::predict() {
    x_last = x;
    update_F();//计算雅可比矩阵
    x = function_f();
    Eigen::MatrixXd F_T = F.transpose();
    P = F * P * F_T + F * Q * F_T;
    //  cout << "P-=" << P<< endl;
    x(3) = x(3) > 1.05 ? 1.05 : x(3);
    x(3) = x(3) < 0.78 ? 0.78 : x(3);
    x(4) = x(4) < 1.884 ? 1.884 : x(4);
    x(4) = x(4) > 2.0 ? 2.0 : x(4);
    return x;
}

Eigen::VectorXd DafuKalman::update(const Eigen::VectorXd& z_meas) {
    Eigen::MatrixXd temp1, temp2, Ht;
    Ht = H.transpose();
    temp1 = H * P * Ht + R;
    temp2 = temp1.inverse(); //(H*P*H'+R)^(-1)
    Eigen::MatrixXd K = P * Ht * temp2; // Kk = P*H'*(H*P*H'+R)^(-1)
    z = H * x;
    x = x + K * (z_meas - z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);//Identity()单位阵
    P = (I - K * H) * P;
    //  cout << "P=" << P << endl;
    x(3) = x(3) > 1.05 ? 1.05 : x(3);
    x(3) = x(3) < 0.78 ? 0.78 : x(3);
    x(4) = x(4) < 1.884 ? 1.884 : x(4);
    x(4) = x(4) > 2.0 ? 2.0 : x(4);
    return x;
}

DafuOutput DafuKalman::extern_kalman_filter(bool is_unsucceeseful, Eigen::VectorXd& _wordCoord) {
    DafuOutput output_temp;
    //判断四种情况
    if (init_flag != 2 && !is_unsucceeseful) {//通过跳变计算角频率，加速收敛
        if (init_flag == 0) {
            init_start_time = rclcpp::Clock().now().seconds();
            first_angle = _wordCoord(0);
            init_flag = 1;
        } else if (init_flag == 1) {
            if (rclcpp::Clock().now().seconds() - init_start_time > 0.25) {
                if (_wordCoord(0) - first_angle > 0) {
                    direction = true;
                } else {
                    direction = false;
                }
                init_flag = 2;
            }
        }
        output_temp.is_success = false;
    } else if (missing_cnt >= DAFUEKF_PREDICT_MAX && !is_unsucceeseful) {//用来给ekf设置初始状态量的前两个值

        x_init(0) = _wordCoord(0);
        x = x_init;                        //初始化状态量，其中前两个用测量位置初始化
        missing_cnt = 0;
    } else if (!is_unsucceeseful) {          //识别成功进行预测更新

        predict();
        if (!is_large_noise(_wordCoord)) {
            x_init = update(_wordCoord);//不断更新初始状态，为下一次丢失作准备
        } else;
        missing_cnt = 0;
        output_temp.is_success = true;
    } else if (is_unsucceeseful && (missing_cnt < DAFUEKF_PREDICT_MAX)) {//没识别到但是在DAFUEKF_PREDICT_MAX步内

        predict();
        missing_cnt++;
        output_temp.is_success = true;
    } else {//丢失时间过长,判定丢失
        missing_cnt = DAFUEKF_PREDICT_MAX;
        output_temp.is_success = false;
    }

    output_temp.ang = x(0);
    output_temp.v = x(1);
    output_temp.psi = x(2);
    output_temp.a = x(3);
    output_temp.w = x(4);

    return output_temp;
}

void DafuKalman::reset_state() {
    init_flag = 0;
    missing_cnt = DAFUEKF_PREDICT_MAX;
}

bool DafuKalman::is_large_noise(const Eigen::VectorXd& _currnet_wordCoord) {
    float noise_x = abs(_currnet_wordCoord(0) - z_abnomal(0));
    if (noise_x > 0.5)
        noise_cnt++;
    else
        noise_cnt = 0;

    if (noise_cnt == 0) {
        z_abnomal = _currnet_wordCoord;
        return false;
    } else if (noise_cnt <= 5)//是噪音
        return true;
    else {
        z_abnomal = _currnet_wordCoord;//是突变
        return false;
    }
}

bool DafuKalman::is_direction_positive() {
    if (direction)
        return true;
    else
        return false;
}

/**
 * @brief IMM_kalman
 * 多模型卡尔曼观测器，自动切换ekf和kf
 * @param is_unsuccessful 当前是否识别到
 * @param _wordCoord 解算装甲板的世界坐标
 * @param _yaw 的装甲板的yaw角
 * @param _number_of_board 目标车辆/前哨站含有的装甲板的数量
 * @param _kalman_CV kf指针
 * @param _ekf_ROTA ekf指针
 * @param _kf_output kf输出结果
 * @param _ekf_output ekf输出结果
 * @return 0：未识别到，1：采用kf，2：采用ekf
 **/
int
IMM_kalman(const bool& is_unsuccessful, const Eigen::VectorXd& _wordCoord, double& _yaw, const int _number_of_board,
           Kalman* _kalman_CV, ExternKalman* _ekf_ROTA, KalmanOutput& _kf_output, EkfOutput& _ekf_output,
           const Eigen::Matrix3d& _rota_matrix3) {
    Eigen::VectorXd wordCoord_temp;
    wordCoord_temp = _wordCoord;//转换为长度单位m
    Eigen::MatrixXd temp = _rota_matrix3;
    _kf_output = _kalman_CV->kalman_filter(is_unsuccessful, wordCoord_temp, temp);
    Eigen::VectorXd ekf_input(4);
    Eigen::VectorXd wordCoord_ekf;
    wordCoord_ekf = _wordCoord;
    wordCoord_ekf(0) = -_wordCoord(0);
    wordCoord_ekf(1) = -_wordCoord(1);//ekf坐标系转换
//    double yaw = _yaw > M_PI ? _yaw - 2 * M_PI : (_yaw < -M_PI ? _yaw + 2 * M_PI : _yaw);
    ekf_input << wordCoord_ekf, _yaw;//小陀罗卡尔曼的坐标系和车体坐标系不太一样
    _ekf_output = _ekf_ROTA->extern_kalman_filter(is_unsuccessful, ekf_input, _number_of_board);
    _yaw = ekf_input(3);
    static bool if_ekf = false;
    if (if_ekf) {
        if (abs(_ekf_output.w) < 0.75) {//ekf和kf切换，通过估计的角速度来判断
            if_ekf = false;
            return 1;
        }
        return 2;
    } else {
        if (abs(_ekf_output.w) > 1.5) {//ekf和kf切换，通过估计的角速度来判断
            if_ekf = true;
            return 2;
        }
        return 1;
    }

//    return 2;
}
