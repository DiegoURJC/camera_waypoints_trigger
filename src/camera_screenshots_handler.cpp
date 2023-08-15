

#include <chrono>
#include <ctime>
#include <cmath>
#include <csignal>
#include <boost/algorithm/string/join.hpp>

#include "camera_waypoints_trigger/camera_screenshots_handler.hpp"
#include "camera_waypoints_trigger/WGS84toCartesian.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/photo.hpp>

const std::string MISSION_DIR_TOKEN = "mission_";
const std::string SLASH_STR = "/";
const std::string COMMA_STR = ",";
const std::string CSV_FILENAME = "metadata.csv";
const std::string HOME_ALT_PARAM = "mission_altitude";
const std::string HOME_GPS_POS = "home_gps_pos";
const std::string LAT_PARAM = "wp_lat";
const std::string LON_PARAM = "wp_lon";
const std::string CSV_NUM_WP_COL_STR = "num_waypoint";
const std::string CSV_RGB_COL_STR = "rgb_img_name";
const std::string CSV_IR_COL_STR = "ir_img_name";
const std::string CSV_X_COL_STR = "x";
const std::string CSV_Y_COL_STR = "y";
const std::string CSV_Z_COL_STR = "z";
const std::string CSV_ROLL_COL_STR = "roll";
const std::string CSV_PITCH_COL_STR = "pitch";
const std::string CSV_YAW_COL_STR = "yaw";
const std::string RGB_IMG_STR = "rgb_img_wp_";
const std::string IR_IMG_STR = "ir_img_wp_";
const std::string PNG_EXTENSION = ".png";
const std::list<std::string> CSV_COL_NAMES = 
  {CSV_NUM_WP_COL_STR, CSV_RGB_COL_STR, CSV_IR_COL_STR, 
   CSV_X_COL_STR, CSV_Y_COL_STR, CSV_Z_COL_STR,
   CSV_ROLL_COL_STR, CSV_PITCH_COL_STR, CSV_YAW_COL_STR};
constexpr int32_t ROLL = 0;
constexpr int32_t PITCH = 1;
constexpr int32_t YAW = 2; 
constexpr int32_t X = 0;
constexpr int32_t Y = 1;
constexpr int32_t Z = 2;


constexpr float NANOSEC_IN_SECS = 10e9f;
constexpr float TOLERANCE = 0.035f;



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
  //  Create directory where everything is going to be saved
  create_mission_dir();

  //  Create CSV file for metadata
  create_mission_CSV();

  initWaypoints();
}


void CameraScreenshotsHandler::initWaypoints()
{
  // Store latitude/longitude from ros2 params
  m_altitude = get_parameter(HOME_ALT_PARAM).as_double();
  const std::vector<double> homeVector = get_parameter(HOME_GPS_POS).as_double_array();
  const std::vector<double> wpLatitude = get_parameter(LAT_PARAM).as_double_array();
  const std::vector<double> wpLongitude = get_parameter(LON_PARAM).as_double_array();

  const std::array<double,2> homePos = {homeVector[X], homeVector[Y]};

  for(std::size_t i = 0; i < wpLatitude.size(); ++i)
  {
    const std::array<double,2> waypoint = {wpLatitude[i], wpLongitude[i]};

    const std::array<double,2> cartesianWp = wgs84::toCartesian(homePos, waypoint);

    m_waypoints.push_back(std::make_pair(cartesianWp[X], cartesianWp[Y]));

    std::cout << std::setprecision(12);

    std::cout << "REFERENCE POINT: (" << homePos[X] << "," << homePos[Y] << 
      ") WAYPOINT: GPS(" << waypoint[X] << "," << waypoint[Y] << 
      ") CART(" << cartesianWp[X] << "," << cartesianWp[Y] << std::endl;

    printf("HOME GPS: (%f,%f) WAYPOINT GPS: (%f,%f) METERS(%f,%f)\n", 
    homePos[X], homePos[Y], waypoint[X], waypoint[Y], cartesianWp[X], cartesianWp[Y]);

  }

  m_numWaypoints = m_waypoints.size();

  RCLCPP_INFO(get_logger(), "NUM WAYPOINTS: %d", m_numWaypoints);

  for(auto const &wp: m_waypoints)
  {
    std::cout << wp.first << " " << wp.second << std::endl;
  }
}


void CameraScreenshotsHandler::ir_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
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
  try
  {
    m_rgbImagePtr = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception %s", e.what());
  }
}


void CameraScreenshotsHandler::quaternion2euler(const std::array<float, 4> &q) noexcept
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

float CameraScreenshotsHandler::checkWaypointHoldTime() const noexcept
{
  const rclcpp::Time nowTimestamp = now();

  const rclcpp::Duration timeDiff = nowTimestamp - m_wpTimestamp;

  const float timeSecs = timeDiff.nanoseconds() / NANOSEC_IN_SECS;


  return timeSecs;

}


void CameraScreenshotsHandler::updateMissionStatus(const std::array<double, 3> &position, const std::array<float, 4> &q)
{
  // Drone is travelling to a waypoint
  if(false == m_onWaypoint)
  {
    // If it is inside the waypoint area
    if((fabs(position[X] - m_waypoints[m_waypointsVisited].first) < ERROR) && 
       (fabs(position[Y] - m_waypoints[m_waypointsVisited].second) < ERROR) &&
       (fabs(position[Z] - m_altitude) < ERROR))
    {
      m_onWaypoint = true;
      m_wpTimestamp = now();
      m_photosTaken = 0;
    }
  }
  // Drone is on a waypoint
  else
  {
    const float timeDiff = checkWaypointHoldTime();
    if((timeDiff >= WP_HOLD_TIME) || (m_photosTaken >= WP_PHOTOS))
    {
      m_onWaypoint = false;
      m_waypointsVisited++;

    }
  }
}


void CameraScreenshotsHandler::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  const std::array<double, 3> position = {msg->x, msg->y, msg->z};
  const std::array<float, 4> q = msg->q;

  quaternion2euler(q);

  if(m_waypointsVisited < m_numWaypoints)
  {
    updateMissionStatus(position, q);

    if(true == m_onWaypoint)
    {
      RCLCPP_INFO(get_logger(), "WAYPOINT DETECTED!!! CHECKING ROLL/PITCH");
      RCLCPP_INFO(get_logger(), "ROLL: %f | PITCH: %f", fabs(m_odom[ROLL]), fabs(m_odom[PITCH]));

      //  If it is parallel to the ground
      if((fabs(m_odom[ROLL]) <= TOLERANCE) && (fabs(m_odom[PITCH]) <= TOLERANCE))
      {
        RCLCPP_INFO(get_logger(), "WAYPOINT READY!, CAPTURING IMAGES...");

        const bool status = storeWaypointData(position, m_odom);

        if(true == status)
        {
          m_photosTaken++;
        }
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(), "NAVIGATING TO WAYPOINT %d: (%f, %f) | CURRENT POSITION: (%f, %f, %f)", 
        m_waypointsVisited+1, m_waypoints[m_waypointsVisited].first, m_waypoints[m_waypointsVisited].second, 
        position[X], position[Y], position[Z]);
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "MISSION ACOMPLISHED! GOING BACK TO HOME...");
    rclcpp::shutdown();
  }


}

bool CameraScreenshotsHandler::storeWaypointData(const std::array<double,3> &position, const std::array<double,3> odom)
{
  const cv::Mat rgbImage = m_rgbImagePtr->image;
  const cv::Mat irImage = m_irImagePtr->image;
  bool retStatus = true;
  const int32_t num_waypoint = m_waypointsVisited+1;
  const int32_t num_photo = m_photosTaken+1;

  if((false == rgbImage.empty()) && (false == irImage.empty()))
  {
    const std::string rgbImageFileName = RGB_IMG_STR + std::to_string(num_waypoint) + "_" + 
                                          std::to_string(num_photo) + PNG_EXTENSION;
    const std::string irImageFileName = IR_IMG_STR + std::to_string(num_waypoint) + "_" + 
                                          std::to_string(num_photo) + PNG_EXTENSION;

    const std::string rgbImagePath = m_missionDir.string() + SLASH_STR + rgbImageFileName;
    const std::string irImagePath = m_missionDir.string() + SLASH_STR + irImageFileName;

    cv::imwrite(rgbImagePath, rgbImage);
    cv::imwrite(irImagePath, irImage);

    const std::list<std::string> fields = 
    {std::to_string(num_waypoint), rgbImageFileName, irImageFileName,
     std::to_string(position[X]), std::to_string(position[Y]), std::to_string(position[Z]),
     std::to_string(odom[ROLL]), std::to_string(odom[PITCH]), std::to_string(odom[YAW])};

    write_CSV_file(fields);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "COULD NOT GET RGB OR IR IMAGES, TRYING AGAIN...");
    retStatus = false;
  }

  return retStatus;
}



int main (int argc, char *argv[])
{
  std::signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CameraScreenshotsHandler>());

  rclcpp::shutdown();

  return 0;
}


