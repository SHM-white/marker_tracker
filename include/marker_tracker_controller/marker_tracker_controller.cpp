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

    modeSubscription = create_subscription<robot_serial::msg::Mode>(
            "/robot/mode",
            10,
            std::bind(&MarkerTrackerController::modeCallback, this, std::placeholders::_1)
    );
}

void MarkerTrackerController::init() {
    ballisticsParams.init(shared_from_this());
    camParams.init(shared_from_this());
    enemyParams.init(shared_from_this());
    kalManParams.init(shared_from_this());
}

void
MarkerTrackerController::detectResultsCallback(const marker_detector::msg::DetectResults::SharedPtr detectResults) {
    TrackerResults trackerResults;
    for (const auto& detectResult: detectResults->detect_results) {
        trackerResults.push_back(trackers[detectResult.id]->track(detectResult));
    }
    int bestTargetId = calculateBestTarget(trackerResults);
    if (bestTargetId != -1) {
        aimPublisher->publish(trackerResults[bestTargetId]);
    }
}

void MarkerTrackerController::modeCallback(const robot_serial::msg::Mode::SharedPtr modeMsg) {
    if (static_cast<Mode>(modeMsg->mode) != mode) {
        if (static_cast<Mode>(modeMsg->mode) == Mode::OUTPOST) {
            trackers[-2]->reinitialize(modeMsg->config);
        }
    }
}

int MarkerTrackerController::calculateBestTarget(const TrackerResults& trackerResults) {
    return (int) trackerResults.size() - 1;
}
