#pragma once


#include <stdint.h>
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <px4_msgs/msg/vehicle_odometry.hpp>


using std::placeholders::_1;



class PeriodicImageCapturer : public rclcpp::Node
{
public:

  PeriodicImageCapturer()
  : Node("periodic_image_capturer"), 
  m_rgbImagePtr(nullptr), 
  m_irImagePtr(nullptr),
  m_photosTaken(1)
  {
    m_RGBSubscriber = create_subscription<sensor_msgs::msg::Image>
    ("/camera/color/image_raw", 50, std::bind(&PeriodicImageCapturer::rgb_callback, this, _1));

    m_IRSubscriber = create_subscription<sensor_msgs::msg::Image>
    ("/camera/infra1/image_rect_raw", 50, std::bind(&PeriodicImageCapturer::ir_callback, this, _1));

    m_odometrySubscriber = create_subscription<px4_msgs::msg::VehicleOdometry>
    ("/fmu/vehicle_odometry/out", 100, std::bind(&PeriodicImageCapturer::odom_callback, this, _1));

    init();

  }

  ~PeriodicImageCapturer() = default;

private:

  void write_CSV_file(const std::list<std::string> &fields);

  void create_mission_CSV();

  void create_mission_dir();

  void init();

  void initWaypoints();

  void ir_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  std::array<double, 3> quaternion2euler(const std::array<float, 4> &q) noexcept;

  double checkTimeElapsed() const noexcept;

  void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  bool storeImages();

  cv_bridge::CvImagePtr m_rgbImagePtr;
  cv_bridge::CvImagePtr m_irImagePtr;

  int32_t m_photosTaken;

  std::array<double, 3> m_odom;

  double m_altitude;

  std::filesystem::path m_missionDir;
  std::string m_csvFilePath;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_RGBSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_IRSubscriber;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_odometrySubscriber;

  std::chrono::time_point<std::chrono::system_clock> m_photoTimestamp;

  static constexpr double IMAGE_CAPTURER_TIMER = 5.0;
};