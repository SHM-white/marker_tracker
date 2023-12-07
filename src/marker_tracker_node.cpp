#include <rclcpp/rclcpp.hpp>

#include <marker_tracker_controller/marker_tracker_controller.h>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerTrackerController>());
    rclcpp::shutdown();
    return 0;
}
