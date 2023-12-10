//
// Created by mijiao on 23-12-7.
//

#include "marker_tracker_controller.h"

MarkerTrackerController::MarkerTrackerController() : Node("marker_tracker_controller") {
    aimPublisher = create_publisher<robot_serial::msg::Aim>("/robot/auto_aim", 5);

    detectResultsSubscription = create_subscription<marker_detector::msg::DetectResults>(
            "/detect_results",
            10,
            std::bind(&MarkerTrackerController::detectResultsCallback, this, std::placeholders::_1)
    );
}

void MarkerTrackerController::init() {
    kalManParams.init(shared_from_this());
}

void
MarkerTrackerController::detectResultsCallback(const marker_detector::msg::DetectResults::SharedPtr detectResults) {
    TrackerResults trackerResults;
    for (const auto& detectResult: detectResults->detect_results) {
        trackerResults.push_back(trackers[detectResult.id]->track(detectResult.pose));
    }
    int bestTargetId = calculateBestTarget(trackerResults);
    if (bestTargetId != -1) {
        aimPublisher->publish(trackerResults[bestTargetId].toRosMsg());
    }
}

void MarkerTrackerController::gimbalCallback(const robot_serial::msg::Gimbal::SharedPtr msg) {
    gimbalYaw = msg->yaw * M_PI / 180;
    gimbalPitch = msg->pitch * M_PI / 180;
    gimbalRoll = msg->roll * M_PI / 180;
    ball_speed = msg->bullet % 128;

//    //markSensor->lock = (bool)(msg->bullet / 128);
//
//    cam2world(0, 0) = cos(gimbalYaw) * cos(gimbalPitch);
//    cam2world(0, 1) = cos(gimbalYaw) * sin(gimbalPitch) * sin(gimbalRoll) - sin(gimbalYaw) * cos(gimbalRoll);
//    cam2world(0, 2) = cos(gimbalYaw) * sin(gimbalPitch) * cos(gimbalRoll) + sin(gimbalYaw) * sin(gimbalRoll);
//    cam2world(1, 0) = cos(gimbalPitch) * sin(gimbalYaw);
//    cam2world(1, 1) = sin(gimbalYaw) * sin(gimbalPitch) * sin(gimbalRoll) + cos(gimbalYaw) * cos(gimbalRoll);
//    cam2world(1, 2) = sin(gimbalYaw) * sin(gimbalPitch) * cos(gimbalRoll) - cos(gimbalYaw) * sin(gimbalRoll);
//    cam2world(2, 0) = -sin(gimbalPitch);
//    cam2world(2, 1) = sin(gimbalRoll) * cos(gimbalPitch);
//    cam2world(2, 2) = cos(gimbalRoll) * cos(gimbalPitch);
//    camCoord << -z - CAMERA_OFFSET_X, x, -y + CAMERA_OFFSET_Z;//将相机坐标系转为云台坐标系
//
//    worldCoord = cam2world * camCoord;//将云台坐标系转化为世界坐标系
}

int MarkerTrackerController::calculateBestTarget(const TrackerResults& trackerResults) {
    return -1;
}
