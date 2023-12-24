//
// Created by mijiao on 23-12-7.
//

#include "tracker.h"

Tracker::Tracker(int _id) : id(_id) {}

Eigen::Vector3d Tracker::toEulerAngles(const geometry_msgs::msg::Quaternion& quaternion) {
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

void Tracker::reinitialize(std::vector<uint8_t>) {

}
