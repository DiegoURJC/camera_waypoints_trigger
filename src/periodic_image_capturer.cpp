

#include <chrono>
#include <ctime>
#include <cmath>
#include <csignal>
#include <boost/algorithm/string/join.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/photo.hpp>

#include "periodic_image_capturer/periodic_image_capturer.hpp"

const std::string MISSION_DIR_TOKEN = "mission_";
const std::string SLASH_STR = "/";
const std::string COMMA_STR = ",";
const std::string CSV_FILENAME = "metadata.csv";
const std::string CSV_X_COL_STR = "x";
const std::string CSV_Y_COL_STR = "y";
const std::string CSV_Z_COL_STR = "z";
const std::string CSV_ROLL_COL_STR = "roll(rad)";
const std::string CSV_PITCH_COL_STR = "pitch(rad)";
const std::string CSV_YAW_COL_STR = "yaw(rad)";
const std::string RGB_IMG_STR = "rgb_img_";
const std::string IR_IMG_STR = "ir_img_";
const std::string PNG_EXTENSION = ".png";
const std::list<std::string> CSV_COL_NAMES = 
  {CSV_X_COL_STR, CSV_Y_COL_STR, CSV_Z_COL_STR,
   CSV_ROLL_COL_STR, CSV_PITCH_COL_STR, CSV_YAW_COL_STR};
constexpr int32_t ROLL = 0;
constexpr int32_t PITCH = 1;
constexpr int32_t YAW = 2; 
constexpr int32_t X = 0;
constexpr int32_t Y = 1;
constexpr int32_t Z = 2;


void signalHandler(int signal)
{
  if(SIGINT == signal)
  {
    rclcpp::shutdown();
  }
}


void PeriodicImageCapturer::write_CSV_file(const std::list<std::string> &fields)
{
  const std::string row = boost::algorithm::join(fields, COMMA_STR);

  std::fstream csvFile(m_csvFilePath, std::ios::in | std::ios::out);

  csvFile.seekp(0, std::ios::end);

  csvFile << row << std::endl;
}


void PeriodicImageCapturer::create_mission_CSV()
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


void PeriodicImageCapturer::create_mission_dir()
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

void PeriodicImageCapturer::init()
{
  //  Create directory where everything is going to be saved
  create_mission_dir();

  //  Create CSV file for metadata
  create_mission_CSV();
}



void PeriodicImageCapturer::ir_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    m_irImagePtr = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception %s", e.what());
  }

  if(checkTimeElapsed() > IMAGE_CAPTURER_TIMER)
  {
    storeImages();
    m_photoTimestamp = std::chrono::system_clock::now();
  }
}


void PeriodicImageCapturer::rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    m_rgbImagePtr = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception %s", e.what());
  }

  if(checkTimeElapsed() > IMAGE_CAPTURER_TIMER)
  {
    storeImages();
    m_photoTimestamp = std::chrono::system_clock::now();
  }
}


std::array<double, 3> PeriodicImageCapturer::quaternion2euler(const std::array<float, 4> &q) noexcept
{
  constexpr int32_t W = 0;
  constexpr int32_t X = 1;
  constexpr int32_t Y = 2;
  constexpr int32_t Z = 3;

  std::array<double, 3> odom;

  // Conversion to roll, pitch, yaw
  odom[ROLL] = std::atan2(2 * (q[W] * q[X] + q[Y] * q[Z]), 
                         1 - 2 * (q[X] * q[X] + q[Y] * q[Y]));
  odom[PITCH] = std::asin(2 * (q[W] * q[Y] - q[Z] * q[X]));
  odom[YAW] = std::atan2(2 * (q[W] * q[Z] + q[X] * q[Y]), 
                         1 - 2 * (q[Y] * q[Y] + q[Z] * q[Z]));

  RCLCPP_INFO(get_logger(), "ROLL: %f(deg) | PITCH: %f(deg) | YAW: %f(deg)", 
    (odom[ROLL]*180.0)/ 3.1415, (odom[PITCH]*180.0)/ 3.1415, (odom[YAW]*180.0)/ 3.1415);

  return odom;
}

double PeriodicImageCapturer::checkTimeElapsed() const noexcept
{
  const auto nowTimestamp = std::chrono::system_clock::now();

  const std::chrono::duration<double> timeElapsed = nowTimestamp - m_photoTimestamp;


  return timeElapsed.count();

}



void PeriodicImageCapturer::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  const std::array<double, 3> pos = {msg->x, msg->y, msg->z};
  const std::array<float, 4> q = msg->q;

  const std::array<double, 3> odom = quaternion2euler(q);

  const std::list<std::string> fields = 
  {std::to_string(pos[X]), std::to_string(pos[Y]), std::to_string(pos[Z]),
  std::to_string(odom[ROLL]), std::to_string(odom[PITCH]), std::to_string(odom[YAW])};

  write_CSV_file(fields);

}

bool PeriodicImageCapturer::storeImages()
{
  bool retStatus;

  if((nullptr != m_rgbImagePtr) && (nullptr != m_irImagePtr))
  {
    const cv::Mat rgbImage = m_rgbImagePtr->image;
    const cv::Mat irImage = m_irImagePtr->image;
    retStatus = true;

    if((false == rgbImage.empty()) && (false == irImage.empty()))
    {
      const std::string rgbImageFileName = RGB_IMG_STR + std::to_string(m_photosTaken) + PNG_EXTENSION;
      const std::string irImageFileName = IR_IMG_STR + std::to_string(m_photosTaken) + PNG_EXTENSION;

      const std::string rgbImagePath = m_missionDir.string() + SLASH_STR + rgbImageFileName;
      const std::string irImagePath = m_missionDir.string() + SLASH_STR + irImageFileName;

      cv::cvtColor(rgbImage, rgbImage, cv::COLOR_BGR2RGB);

      cv::imwrite(rgbImagePath, rgbImage);
      cv::imwrite(irImagePath, irImage);

      m_photosTaken++;      

      
      m_rgbImagePtr = nullptr;
      m_irImagePtr = nullptr;
    }
    else
    {
      RCLCPP_WARN(get_logger(), "COULD NOT GET RGB OR IR IMAGES, TRYING AGAIN...");
      retStatus = false;
    }
  }
  else
  {
    retStatus = false;
  }


  return retStatus;
}



int main (int argc, char *argv[])
{
  std::signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PeriodicImageCapturer>());

  rclcpp::shutdown();

  return 0;
}


