#pragma once


#include <stdint.h>
#include <filesystem>
#include <fstream>

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
  m_photosTaken(0),
  m_odom({}),
  m_altitude(0.0),
  m_onWaypoint(false)
  {
    declare_parameter("mission_altitude");
    declare_parameter("home_gps_pos");
    declare_parameter("wp_lat");
    declare_parameter("wp_lon");


    m_RGBSubscriber = create_subscription<sensor_msgs::msg::Image>
    ("/camera/color/image_raw", 50, std::bind(&CameraScreenshotsHandler::rgb_callback, this, _1));

    m_IRSubscriber = create_subscription<sensor_msgs::msg::Image>
    ("/camera/infra1/image_rect_raw", 50, std::bind(&CameraScreenshotsHandler::ir_callback, this, _1));

    m_odometrySubscriber = create_subscription<px4_msgs::msg::VehicleOdometry>
    ("/fmu/vehicle_odometry/out", 100, std::bind(&CameraScreenshotsHandler::odom_callback, this, _1));

    init();

    m_onWaypoint = true;
    m_wpTimestamp = std::chrono::system_clock::now();

  }

  ~CameraScreenshotsHandler() = default;

private:

  void write_CSV_file(const std::list<std::string> &fields);

  void create_mission_CSV();

  void create_mission_dir();

  void init();

  void initWaypoints();

  void ir_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void quaternion2euler(const std::array<float, 4> &q) noexcept;

  double checkWaypointHoldTime() const noexcept;

  void updateMissionStatus(const std::array<double, 3> &position, const std::array<float, 4> &q);

  void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  bool storeWaypointData(const std::array<double,3> &position, const std::array<double,3> odom);

  cv_bridge::CvImagePtr m_rgbImagePtr;
  cv_bridge::CvImagePtr m_irImagePtr;

  std::vector<std::pair<double, double>> m_waypoints;

  int32_t m_numWaypoints;

  int32_t m_waypointsVisited;

  int32_t m_photosTaken;

  std::array<double, 3> m_odom;

  double m_altitude;

  std::filesystem::path m_missionDir;
  std::string m_csvFilePath;

  std::array<double,3> m_homePosNED;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_RGBSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_IRSubscriber;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_odometrySubscriber;

  bool m_onWaypoint;

  std::chrono::time_point<std::chrono::system_clock> m_wpTimestamp;

  static constexpr int32_t ERROR = 1;
  static constexpr int32_t WP_PHOTOS = 5;
  static constexpr float WP_HOLD_TIME = 20.0f;
};