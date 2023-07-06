

#include <chrono>
#include <ctime>
#include <cmath>
#include <csignal>
#include <boost/algorithm/string/join.hpp>

#include "camera_waypoints_trigger/camera_screenshots_handler.hpp"
#include <opencv2/highgui.hpp>
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
const std::string RGB_IMG_STR = "rgb_img_wp_";
const std::string IR_IMG_STR = "ir_img_wp_";
const std::string PNG_EXTENSION = ".png";
const std::list<std::string> CSV_COL_NAMES = {CSV_RGB_COL_STR, CSV_IR_COL_STR, CSV_LAT_COL_STR, CSV_LON_COL_STR, CSV_YAW_COL_STR};
constexpr int32_t ROLL = 0;
constexpr int32_t PITCH = 1;
constexpr int32_t YAW = 2; 
constexpr float TOLERANCE = 0.035;


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

  std::fstream csvFile(m_csvFilePath, std::ios::in | std::ios::out);

  csvFile.seekp(0, std::ios::end);

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
        std::filesystem::permissions(m_missionDir, 
                                      std::filesystem::perms::owner_all | std::filesystem::perms::group_read | 
                                      std::filesystem::perms::group_write | std::filesystem::perms::others_read);

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


void CameraScreenshotsHandler::quaternion2euler(const std::array<float, 4> &q) 
{
  constexpr int32_t W = 0;
  constexpr int32_t X = 1;
  constexpr int32_t Y = 2;
  constexpr int32_t Z = 3;

  // Conversion to roll, pitch, yaw
  m_odom[ROLL] = std::atan2(2 * (q[W] * q[X] + q[Y] * q[Z]), 
                         1 - 2 * (q[X] * q[X] + q[Y] * q[Y]));
  m_odom[PITCH] = std::asin(2 * (q[W] * q[Y] - q[Z] * q[X]));
  m_odom[YAW] = std::atan2(2 * (q[W] * q[Z] + q[X] * q[Y]), 
                         1 - 2 * (q[Y] * q[Y] + q[Z] * q[Z]));

  RCLCPP_INFO(get_logger(), "ROLL: %f(deg) | PITCH: %f(deg) | YAW: %f(deg)", (m_odom[0]*180.0)/ 3.1415, (m_odom[1]*180.0)/ 3.1415, (m_odom[2]*180.0)/ 3.1415);
}


void CameraScreenshotsHandler::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  const std::array<float, 4> q = msg->q;

  quaternion2euler(q);

}

bool CameraScreenshotsHandler::storeWaypointData(const int32_t &lat, const int32_t &lon, const float &yaw)
{
  const cv::Mat rgbImage = m_rgbImagePtr->image;
  const cv::Mat irImage = m_irImagePtr->image;
  bool retStatus = true;

  if((false == rgbImage.empty()) && (false == irImage.empty()))
  {
    const std::string rgbImageFileName = RGB_IMG_STR + std::to_string(m_waypointsVisited+1) + PNG_EXTENSION;
    const std::string irImageFileName = IR_IMG_STR + std::to_string(m_waypointsVisited+1) + PNG_EXTENSION;

    const std::string rgbImagePath = m_missionDir.string() + SLASH_STR + rgbImageFileName;
    const std::string irImagePath = m_missionDir.string() + SLASH_STR + irImageFileName;

    cv::imwrite(rgbImagePath, rgbImage);
    cv::imwrite(irImagePath, irImage);

    const std::list<std::string> fields = {rgbImageFileName, irImageFileName, std::to_string(lat), 
                                            std::to_string(lon), std::to_string(yaw)};

    write_CSV_file(fields);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "COULD NOT GET RGB OR IR IMAGES, TRYING AGAIN...");
    retStatus = false;
  }


  return retStatus;
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
    if(((msg->lat - m_waypoints[m_waypointsVisited].first) < ERROR) && 
       ((msg->lon - m_waypoints[m_waypointsVisited].second) < ERROR))
    {
      RCLCPP_INFO(get_logger(), "WAYPOINT DETECTED!!! CHECKING ROLL/PITCH");
      RCLCPP_INFO(get_logger(), "ROLL: %s | PITCH: %s", fabs(m_odom[ROLL]), fabs(m_odom[PITCH]));

      //  If it is parallel to the ground
      if((fabs(m_odom[ROLL]) <= TOLERANCE) && (fabs(m_odom[PITCH]) <= TOLERANCE))
      {
        RCLCPP_INFO(get_logger(), "WAYPOINT READY!, CAPTURING IMAGES...");

        const bool status = storeWaypointData(msg->lat, msg->lon, m_odom[YAW]);

        if(true == status)
        {
          m_waypointsVisited++;
        }
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


