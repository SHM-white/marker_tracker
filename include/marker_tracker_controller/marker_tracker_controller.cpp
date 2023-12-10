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

int MarkerTrackerController::calculateBestTarget(TrackerResults& trackerResults) {
    return -1;
}
