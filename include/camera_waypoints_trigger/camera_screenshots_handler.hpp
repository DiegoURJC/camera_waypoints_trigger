#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/highgui.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/photo.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <stdint.h>

using std::placeholders::_1;

class CameraScreenshotsHandler : public rclcpp::Node
{
public:

  CameraScreenshotsHandler()
  : Node("camera_screenshots_handler"), 
  m_rgbImagePtr(nullptr), 
  m_irImagePtr(nullptr),
  m_waypoints({})
  {
    declare_parameter("wp_lat");
    declare_parameter("wp_lon");

    m_RGBSubscriber = create_subscription<sensor_msgs::msg::Image>
    ("/camera/color/image_raw", 10, std::bind(&CameraScreenshotsHandler::rgb_callback, this, _1));

    m_IRSubscriber = create_subscription<sensor_msgs::msg::Image>
    ("/camera/infra1/image_rect_raw", 10, std::bind(&CameraScreenshotsHandler::ir_callback, this, _1));

    m_GPSSubscriber = create_subscription<px4_msgs::msg::VehicleGpsPosition>
    ("/fmu/vehicle_gps_position/out", 10, std::bind(&CameraScreenshotsHandler::gps_callback, this, _1));

    init();

  }

  ~CameraScreenshotsHandler() = default;

private:

  inline void init()
  {
    const std::vector<int64_t> wp_latitude = get_parameter("wp_lat").as_integer_array();
    const std::vector<int64_t> wp_longitude = get_parameter("wp_lon").as_integer_array();

    for(std::size_t i = 0; i < wp_latitude.size(); ++i)
    {
      m_waypoints.push_back(std::make_pair(wp_latitude[i], wp_longitude[i]));
    }

    m_numWaypoints = m_waypoints.size();

    RCLCPP_INFO(get_logger(), "NUM WAYPOINTS: %d", m_numWaypoints);

  }

  inline void ir_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "IR CALLBACK");

    try
    {
      m_irImagePtr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception %s", e.what());
    }
  }

  inline void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "RGB CALLBACK");
    try
    {
      m_rgbImagePtr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception %s", e.what());
    }
  }

  inline void gps_callback(const px4_msgs::msg::VehicleGpsPosition::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "GPS CALLBACK --> LAT: %d, LON: %d", msg->lat, msg->lon);
  }

  cv_bridge::CvImagePtr m_rgbImagePtr;
  cv_bridge::CvImagePtr m_irImagePtr;


  bool m_takeScreenshot;

  std::vector<std::pair<int64_t, int64_t>> m_waypoints;

  int32_t m_numWaypoints;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_RGBSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_IRSubscriber;
  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr m_GPSSubscriber;
};