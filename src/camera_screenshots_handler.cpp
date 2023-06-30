

#include <chrono>
#include <ctime>
#include <cmath>
#include <csignal>

#include "camera_waypoints_trigger/camera_screenshots_handler.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/photo.hpp>


void signalHandler(int signal)
{
  if(SIGINT == signal)
  {
    rclcpp::shutdown();
  }
}


void CameraScreenshotsHandler::create_mission_CSV()
{
  const std::string fileName = "metadata.csv";
  const std::string filePath = m_missionDir.string() + fileName;

  const std::filesystem::path csvPath = std::filesystem::path(filePath);
}

void CameraScreenshotsHandler::create_mission_dir()
{
  const std::string missionStr = "mission_";
  char timeBuffer[255];

  // Get current time and date for dir name
  const auto now = std::chrono::system_clock::now();

  const std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

  std::strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d_%H-%M-%S", std::localtime(&currentTime));

  const std::string dirPath = missionStr + timeBuffer;

  m_missionDir = std::filesystem::path(dirPath);

  // Create directory
  try
  {
    if(std::filesystem::exists(m_missionDir))
    {
      RCLCPP_WARN(get_logger(), "DIR '%s' ALREADY EXISTS!", dirPath);
    }
    else
    {
      const bool dirCreated = std::filesystem::create_directory(m_missionDir);

      if(true == dirCreated)
      {
        std::filesystem::permissions(m_missionDir, std::filesystem::perms::owner_all | std::filesystem::perms::group_read | 
                                                   std::filesystem::perms::group_write | std::filesystem::perms::others_read);
        RCLCPP_INFO(get_logger(), "MISSION DIR '%s' CREATED", dirPath);
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "ERROR CREATING DIR '%s', ABORTING...", dirPath);
        std::raise(SIGINT);
      }
    }
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "ERROR: Exception caught creating dir --> %s", e.what());
    std::raise(SIGINT);
  }

}


void CameraScreenshotsHandler::init()
{
  // Store latitude/longitude from ros2 params
  const std::vector<int64_t> wpLatitude = get_parameter("wp_lat").as_integer_array();
  const std::vector<int64_t> wpLongitude = get_parameter("wp_lon").as_integer_array();

  for(std::size_t i = 0; i < wpLatitude.size(); ++i)
  {
    m_waypoints.push_back(std::make_pair(wpLatitude[i], wpLongitude[i]));
  }

  m_numWaypoints = m_waypoints.size();

  RCLCPP_INFO(get_logger(), "NUM WAYPOINTS: %d", m_numWaypoints);

  //  Create directory where everything is going to be saved
  create_mission_dir();

}


void CameraScreenshotsHandler::ir_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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


void CameraScreenshotsHandler::rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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

void CameraScreenshotsHandler::quaternion2euler(const std::array<float, 4> &q, const std::array<float, 4> &qOffset) 
{
  // Apply offset adjustment
  float correctedQ[4] = {q[0] - qOffset[0], q[1] - qOffset[1], q[2] - qOffset[2], q[3] - qOffset[3]};

  // Normalize values
  const float norm = std::sqrt(correctedQ[0]*correctedQ[0] + correctedQ[1]*correctedQ[1] + correctedQ[2]*correctedQ[2] + correctedQ[3]*correctedQ[3]);
  correctedQ[0] /= norm;
  correctedQ[1] /= norm;
  correctedQ[2] /= norm;
  correctedQ[3] /= norm;

  // Conversion to roll, pitch, yaw
  m_odom[0] = std::atan2(2 * (correctedQ[3] * correctedQ[0] + correctedQ[1] * correctedQ[2]), 1 - 2 * (correctedQ[0] * correctedQ[0] + correctedQ[1] * correctedQ[1]));
  m_odom[1] = std::asin(2 * (correctedQ[3] * correctedQ[1] - correctedQ[2] * correctedQ[0]));
  m_odom[2] = std::atan2(2 * (correctedQ[3] * correctedQ[2] + correctedQ[0] * correctedQ[1]), 1 - 2 * (correctedQ[1] * correctedQ[1] + correctedQ[2] * correctedQ[2]));
}


void CameraScreenshotsHandler::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  const std::array<float, 4> q = msg->q;
  const std::array<float, 4> qOffset = msg->q_offset;


  quaternion2euler(q, qOffset);

}

void CameraScreenshotsHandler::gps_callback(const px4_msgs::msg::VehicleGpsPosition::SharedPtr msg)
{

  if(m_waypointsVisited < m_numWaypoints)
  {
    RCLCPP_INFO(get_logger(), "WE VISITING WAYPOINT %d --> LAT: %d | LON: %d", m_waypointsVisited, 
    m_waypoints[m_waypointsVisited].first, m_waypoints[m_waypointsVisited].second);

    RCLCPP_INFO(get_logger(), "LAT ERROR: %d | LON ERROR: %d", 
    (msg->lat - m_waypoints[m_waypointsVisited].first), (msg->lon - m_waypoints[m_waypointsVisited].second));

    if(((msg->lat - m_waypoints[m_waypointsVisited].first) < ERROR) && ((msg->lon - m_waypoints[m_waypointsVisited].second) < ERROR))
    {
      RCLCPP_INFO(get_logger(), "WAYPOINT DETECTED!!!!");
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "MISSION ACOMPLISHED! GOING BACK TO HOME...");
  }
}


int main (int argc, char *argv[])
{
  std::signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CameraScreenshotsHandler>());

  rclcpp::shutdown();

  return 0;
}


