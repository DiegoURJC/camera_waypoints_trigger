#include "camera_waypoints_trigger/camera_screenshots_handler.hpp"



int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CameraScreenshotsHandler>());

  rclcpp::shutdown();

  return 0;
}


