

#include <chrono>
#include <ctime>
#include <cmath>
#include <csignal>
#include <boost/algorithm/string/join.hpp>

#include "camera_waypoints_trigger/camera_screenshots_handler.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/photo.hpp>

const std::string MISSION_DIR_TOKEN = "mission_";
const std::string SLASH_STR = "/";
const std::string COMMA_STR = ",";
const std::string CSV_FILENAME = "metadata.csv";
const std::string LAT_PARAM = "wp_lat";
const std::string LON_PARAM = "wp_lon";
const std::string CSV_RGB_COL_STR = "rgb_img_name";
const std::string CSV_IR_COL_STR = "ir_img_name";
const std::string CSV_LAT_COL_STR = "latitude";
const std::string CSV_LON_COL_STR = "longitude";
const std::string CSV_YAW_COL_STR = "yaw";
const std::string RGB_IMG_STR = "rgb_img_";
const std::string IR_IMG_STR = "ir_img_";
const std::list<std::string> CSV_COL_NAMES = {CSV_RGB_COL_STR, CSV_IR_COL_STR, CSV_LAT_COL_STR, CSV_LON_COL_STR, CSV_YAW_COL_STR};
constexpr int32_t ROLL = 0;
constexpr int32_t PITCH = 1;
constexpr int32_t YAW = 2; 
constexpr float TOLERANCE = 1.0e-3;


void signalHandler(int signal)
{
  if(SIGINT == signal)
  {
    rclcpp::shutdown();
  }
}


void CameraScreenshotsHandler::write_CSV_file(const std::list<std::string> &fields)
{
  const std::string row = boost::algorithm::join(fields, COMMA_STR);

  std::ofstream csvFile(m_csvFilePath);

  csvFile << row << std::endl;
}


void CameraScreenshotsHandler::create_mission_CSV()
{
  m_csvFilePath = m_missionDir.string() + SLASH_STR + CSV_FILENAME;

  std::ofstream csvFile(m_csvFilePath);

  if (csvFile) 
  {
    RCLCPP_INFO(get_logger(), "CSV FILE '%s' CREATED", m_csvFilePath.c_str());
  } 
  else 
  {
    RCLCPP_ERROR(get_logger(), "ERROR WHILE CREATING CSV FILE '%s', ABORTING...", m_csvFilePath.c_str());
    std::raise(SIGINT);
  }

  write_CSV_file(CSV_COL_NAMES);
}


void CameraScreenshotsHandler::create_mission_dir()
{
  char timeBuffer[255];

  // Get current time and date for dir name
  const auto now = std::chrono::system_clock::now();

  const std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

  std::strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d_%H-%M-%S", std::localtime(&currentTime));

  const std::string dirPath = MISSION_DIR_TOKEN + timeBuffer;

  m_missionDir = std::filesystem::path(dirPath);

  // Create directory
  try
  {
    if(std::filesystem::exists(m_missionDir))
    {
      RCLCPP_WARN(get_logger(), "DIR '%s' ALREADY EXISTS!", m_missionDir.string().c_str());
    }
    else
    {
      const bool dirCreated = std::filesystem::create_directory(m_missionDir);

      if(true == dirCreated)
      {
        std::filesystem::permissions(m_missionDir, std::filesystem::perms::owner_all | std::filesystem::perms::group_read | 
                                                   std::filesystem::perms::group_write | std::filesystem::perms::others_read);
        // std::cout << "MISSION DIR '" << m_missionDir.generic_string() << "' CREATED" << std::endl;
        RCLCPP_INFO(get_logger(), "MISSION DIR '%s' CREATED", m_missionDir.string().c_str());
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "ERROR CREATING DIR '%s', ABORTING...", m_missionDir.string().c_str());
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
  const std::vector<int64_t> wpLatitude = get_parameter(LAT_PARAM).as_integer_array();
  const std::vector<int64_t> wpLongitude = get_parameter(LON_PARAM).as_integer_array();

  for(std::size_t i = 0; i < wpLatitude.size(); ++i)
  {
    m_waypoints.push_back(std::make_pair(wpLatitude[i], wpLongitude[i]));
  }

  m_numWaypoints = m_waypoints.size();

  RCLCPP_INFO(get_logger(), "NUM WAYPOINTS: %d", m_numWaypoints);

  //  Create directory where everything is going to be saved
  create_mission_dir();

  //  Create CSV file for metadata
  create_mission_CSV();

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

void CameraScreenshotsHandler::storeWaypointData(const int32_t &lat, const int32_t &lon, const float &yaw)
{
  const cv::Mat rgbImage = m_rgbImagePtr->image;
  const cv::Mat irImage = m_irImagePtr->image;

  const std::string rgbImageStr = m_missionDir.string() + SLASH_STR + RGB_IMG_STR + std::to_string(m_waypointsVisited);
  const std::string irImageStr = m_missionDir.string() + SLASH_STR + IR_IMG_STR + std::to_string(m_waypointsVisited);

  cv::imwrite(rgbImageStr, rgbImage);
  cv::imwrite(irImageStr, irImage);

  const std::list<std::string> fields = {rgbImageStr, irImageStr, std::to_string(lat), std::to_string(lon), std::to_string(yaw)};

  write_CSV_file(fields);
  }


void CameraScreenshotsHandler::gps_callback(const px4_msgs::msg::VehicleGpsPosition::SharedPtr msg)
{

  if(m_waypointsVisited < m_numWaypoints)
  {
    RCLCPP_INFO(get_logger(), "WE VISITING WAYPOINT %d --> LAT: %d | LON: %d", m_waypointsVisited, 
    m_waypoints[m_waypointsVisited].first, m_waypoints[m_waypointsVisited].second);

    RCLCPP_INFO(get_logger(), "LAT ERROR: %d | LON ERROR: %d", 
    (msg->lat - m_waypoints[m_waypointsVisited].first), (msg->lon - m_waypoints[m_waypointsVisited].second));

    // If it is inside the waypoint area
    if(((msg->lat - m_waypoints[m_waypointsVisited].first) < ERROR) && ((msg->lon - m_waypoints[m_waypointsVisited].second) < ERROR))
    {
      RCLCPP_INFO(get_logger(), "WAYPOINT DETECTED!!! CHECKING ROLL/PITCH");
      RCLCPP_INFO(get_logger(), "ROLL: %s | PITCH: %s", fabs(m_odom[ROLL]), fabs(m_odom[PITCH]));

      //  If it is parallel to the ground
      if((fabs(m_odom[ROLL]) <= TOLERANCE) && (fabs(m_odom[PITCH]) <= TOLERANCE))
      {
        RCLCPP_INFO(get_logger(), "WAYPOINT READY!, CAPTURING IMAGES...");

        storeWaypointData(msg->lat, msg->lon, m_odom[YAW]);

        m_waypointsVisited++;
      }
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "MISSION ACOMPLISHED! GOING BACK TO HOME...");
    rclcpp::shutdown();
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


