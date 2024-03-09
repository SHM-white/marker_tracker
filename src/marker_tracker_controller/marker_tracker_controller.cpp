//
// Created by mijiao on 23-12-7.
//

#include "marker_tracker_controller/marker_tracker_controller.h"

using namespace std::chrono_literals;

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

    kalmanTimer = create_wall_timer(5ms, [this] { kalmanCallback(); });
}

void MarkerTrackerController::init() {
    ballisticsParams.init(shared_from_this());
    camParams.init(shared_from_this());
    kalManParams.init(shared_from_this());
    for (int i = 1; i <= 8; i++) {
        trackers[i] = std::make_shared<ArmorTracker>(i);
    }
    trackers[0] = std::make_shared<BuffTracker>();
}

void
MarkerTrackerController::detectResultsCallback(
        const marker_detector::msg::DetectResults::ConstSharedPtr &detectResults) {
    lastDetectResults = detectResults;
}

void MarkerTrackerController::modeCallback(const robot_serial::msg::Mode::ConstSharedPtr &modeMsg) {
    if (static_cast<Mode>(modeMsg->mode) != mode) {
        if (static_cast<Mode>(modeMsg->mode) == Mode::OUTPOST) {
            trackers[7]->reinitialize(modeMsg->config);
        }
    }
}

int MarkerTrackerController::calculateBestTarget(const TrackerResults &trackerResults) {
    //临时使用，锁序号最小的
    for (size_t i = 0; i < trackerResults.size(); i++) {
        if (trackerResults[i].success) {
            return (int) i;
        }
    }
    return -1;
}

void MarkerTrackerController::kalmanCallback() {
    if (!lastDetectResults) {
        return;
    }
    TrackerResults trackerResults(9);
    for (const auto &detectResult: lastDetectResults->detect_results) {
        trackerResults.push_back(trackers[detectResult.id]->track(detectResult));
    }
    int bestTargetId = calculateBestTarget(trackerResults);
    if (bestTargetId != -1) {
        aimPublisher->publish(trackerResults[bestTargetId]);
    }
}
