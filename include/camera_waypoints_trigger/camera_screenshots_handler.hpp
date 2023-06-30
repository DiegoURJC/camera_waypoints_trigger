#pragma once


#include <stdint.h>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>


using std::placeholders::_1;


class CameraScreenshotsHandler : public rclcpp::Node
{
public:

  CameraScreenshotsHandler()
  : Node("camera_screenshots_handler"), 
  m_rgbImagePtr(nullptr), 
  m_irImagePtr(nullptr),
  m_waypoints({}),
  m_numWaypoints(0),
  m_waypointsVisited(0),
  m_odom({})
  {
    declare_parameter("wp_lat");
    declare_parameter("wp_lon");

    m_RGBSubscriber = create_subscription<sensor_msgs::msg::Image>
    ("/camera/color/image_raw", 30, std::bind(&CameraScreenshotsHandler::rgb_callback, this, _1));

    m_IRSubscriber = create_subscription<sensor_msgs::msg::Image>
    ("/camera/infra1/image_rect_raw", 30, std::bind(&CameraScreenshotsHandler::ir_callback, this, _1));

    m_GPSSubscriber = create_subscription<px4_msgs::msg::VehicleGpsPosition>
    ("/fmu/vehicle_gps_position/out", 100, std::bind(&CameraScreenshotsHandler::gps_callback, this, _1));

    m_odometrySubscriber = create_subscription<px4_msgs::msg::VehicleOdometry>
    ("/fmu/vehicle_odometry/out", 100, std::bind(&CameraScreenshotsHandler::odom_callback, this, _1));

    init();

  }

  ~CameraScreenshotsHandler() = default;

private:

  void create_mission_CSV();

  void create_mission_dir();

  void init();

  void ir_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void quaternion2euler(const std::array<float, 4> &q, const std::array<float, 4> &q_offset); 

  void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  void gps_callback(const px4_msgs::msg::VehicleGpsPosition::SharedPtr msg);

  cv_bridge::CvImagePtr m_rgbImagePtr;
  cv_bridge::CvImagePtr m_irImagePtr;


  std::vector<std::pair<int64_t, int64_t>> m_waypoints;

  int32_t m_numWaypoints;

  int32_t m_waypointsVisited;

  std::array<float, 3> m_odom;

  std::filesystem::path m_missionDir;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_RGBSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_IRSubscriber;
  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr m_GPSSubscriber;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_odometrySubscriber;

  const int32_t ERROR = 100;
};