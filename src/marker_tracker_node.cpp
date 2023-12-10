#include <rclcpp/rclcpp.hpp>

#include <marker_tracker_controller/marker_tracker_controller.h>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerTrackerController>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
