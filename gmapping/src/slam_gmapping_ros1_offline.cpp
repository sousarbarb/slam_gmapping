#include "slam_gmapping_ros1_offline.h"

#include <fcntl.h>
#include <sys/ioctl.h>

#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

// ROS
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_msgs/TFMessage.h>

SLAMGMappingROS1Offline::SLAMGMappingROS1Offline(const ParamOffline& param)
    : SLAMGMappingROS1API::SLAMGMappingROS1API(), param_offline_(param)
{
  ros::Time::init();

  tf2_buffer_.setUsingDedicatedThread(true);

  // Print offline parametrization
  std::stringstream str;

  for (const std::string& bag_filename : param_offline_.bags)
  {
    str << "- " << bag_filename << std::endl;
  }

  ROS_INFO("[%s] bag files :\n%s", ros::this_node::getName().c_str(),
           str.str().c_str());
  ROS_INFO("[%s] scan topic: %s", ros::this_node::getName().c_str(),
           param_offline_.scan_topic.c_str());

  // Log file processing
  if (param_offline_.enable_log)
  {
    if (param_offline_.log_filename.empty())
    {
      throw std::runtime_error(
          "SLAMGMappingROS1Offline::SLAMGMappingROS1Offline | empty filename "
          "when log enabled");
    }

    std::string log_file_pose;

    try
    {
      std::filesystem::path log_file_path(param_offline_.log_filename);

      log_file_pose = log_file_path.stem().string() + "_gmapping_laser" +
                      log_file_path.extension().string();

      ROS_INFO("[%s] log file  : %s", ros::this_node::getName().c_str(),
               log_file_pose.c_str());
    }
    catch (const std::filesystem::filesystem_error& e)
    {
      throw std::runtime_error(
          "SLAMGMappingROS1Offline::SLAMGMappingROS1Offline | Error resolving "
          "paths for log files");
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error(
          "SLAMGMappingROS1Offline::SLAMGMappingROS1Offline | Error when "
          "processing paths for log files");
    }
    catch (...)
    {
      throw std::runtime_error(
          "SLAMGMappingROS1Offline::SLAMGMappingROS1Offline | Unknown error "
          "when processing paths for log files");
    }

    validateAndCreatePath(log_file_pose);

    try
    {
      log_file_pose_ = std::ofstream(log_file_pose);

      if (!log_file_pose_.is_open())
      {
        throw std::runtime_error(
            "SLAMGMappingROS1Offline::SLAMGMappingROS1Offline | file (" +
            param_offline_.log_filename + ") for pose data not opened");
      }
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error(
          "SLAMGMappingROS1Offline::SLAMGMappingROS1Offline | error when "
          "opening the log file (" +
          log_file_pose + "): " + e.what());
    }
  }
  else
  {
    ROS_INFO("[%s] log file  : not enabled", ros::this_node::getName().c_str());
  }

  std::cout << std::endl;

  // ROS API
  sst_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

  scan_filter_ =
      std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(
          tf2_buffer_, odom_frame_, 10, nh_priv_);

  scan_filter_->registerCallback(&SLAMGMappingROS1API::laserCallback,
                                 static_cast<SLAMGMappingROS1API*>(this));
}

SLAMGMappingROS1Offline::~SLAMGMappingROS1Offline()
{
  if (param_offline_.enable_log && log_file_pose_.is_open())
  {
    log_file_pose_.close();
  }

  for (const std::shared_ptr<rosbag::Bag>& bag : bags_)
  {
    if (bag->isOpen())
    {
      ROS_INFO("[%s] Closing %s", ros::this_node::getName().c_str(),
               bag->getFileName().c_str());

      bag->close();
    }
  }

  restoreTerminal();
}

void SLAMGMappingROS1Offline::run()
{
  std::cout << std::endl;

  setupTerminal();

  auto start = std::chrono::high_resolution_clock::now();

  for (const std::string& filename : param_offline_.bags)
  {
    ROS_INFO("[%s] Opening %s", ros::this_node::getName().c_str(),
             filename.c_str());

    try
    {
      std::shared_ptr<rosbag::Bag> bag = std::make_shared<rosbag::Bag>();

      bag->open(filename, rosbag::bagmode::Read);

      bags_.push_back(bag);
    }
    catch (rosbag::BagException& e)
    {
      std::stringstream error;

      error << "Error when opening the ROS bag file (filename: " << filename
            << "; error: " << e.what() << ")";

      throw std::runtime_error(error.str());
    }
  }

  rosbag::View full_view;

  for (const std::shared_ptr<rosbag::Bag>& bag : bags_)
  {
    full_view.addQuery(*bag);
  }

  const ros::Time full_initial_time = full_view.getBeginTime();
  const ros::Time initial_time =
      full_initial_time + ros::Duration(param_offline_.time_start);
  ros::Time finish_time = ros::TIME_MAX;

  ros::Duration bag_length;

  ROS_INFO("[%s] Start  time (s): %.9lf", ros::this_node::getName().c_str(),
           initial_time.toSec());

  if (param_offline_.has_duration)
  {
    finish_time = initial_time + ros::Duration(param_offline_.time_duration);
    bag_length = finish_time - initial_time;

    ROS_INFO("[%s] Finish time (s): %.9lf", ros::this_node::getName().c_str(),
             finish_time.toSec());
    ROS_INFO("[%s] Total  time (s): %.9lf\n", ros::this_node::getName().c_str(),
             bag_length.toSec());
  }
  else
  {
    bag_length = full_view.getEndTime() - initial_time;

    ROS_INFO("[%s] Finish time (s): %.9lf (end of the bags)",
             ros::this_node::getName().c_str(), full_view.getEndTime().toSec());
    ROS_INFO("[%s] Total  time (s): %.9lf\n", ros::this_node::getName().c_str(),
             bag_length.toSec());
  }

  rosbag::View view;

  for (const std::shared_ptr<rosbag::Bag>& bag : bags_)
  {
    view.addQuery(*bag, initial_time, finish_time);
  }

  std::cout << "\033[31m"
            << "Press SPACE to pause/resume processing, 'q' to quit..."
            << "\033[0m" << std::endl;

  for (rosbag::MessageInstance const& msg : view)
  {
    while (true)
    {
      char key = readTerminalKey();

      if (key == ' ')
      {
        paused_ = !paused_;

        std::cout << std::endl << std::flush;

        printTime(msg.getTime(), msg.getTime() - initial_time, bag_length);

        std::cout << "\033[31m" << (paused_ ? "[PAUSED ]" : "[RESUMED]")
                  << " Press SPACE to pause/resume processing, 'q' to quit..."
                  << "\033[0m" << std::endl;
      }
      else if (key == 'q' || key == 'Q')
      {
        std::cout << "Processing stopped by user" << std::endl;
        goto exit_loop;
      }

      if (!paused_)
      {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (msg.instantiate<sensor_msgs::LaserScan>() != nullptr)
    {
      if (msg.getTopic() != param_offline_.scan_topic)
      {
        continue;
      }

      sensor_msgs::LaserScanPtr laser_msg =
          msg.instantiate<sensor_msgs::LaserScan>();

      scan_filter_->add(laser_msg);
    }
    else if (msg.instantiate<tf2_msgs::TFMessage>() != nullptr)
    {
      tf2_msgs::TFMessagePtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();

      bool is_static = (msg.getTopic().compare("/tf_static") == 0);

      for (const geometry_msgs::TransformStamped& transf : tf_msg->transforms)
      {
        tf2_buffer_.setTransform(transf, ros::this_node::getName(), is_static);
      }
    }

    ros::spinOnce();
  }

// Exit loop if 'q' was pressed while processing the ROS bags
exit_loop:

  auto end = std::chrono::high_resolution_clock::now();

  std::cout << std::endl << std::flush;

  ROS_INFO(
      "\n\n"
      "[%s] Finished processing the ROS bags.\n"
      "Elapsed time (s): %.3lf\n"
      "ros::spin to allow rosrun map_server map_saver OR rviz "
      "visualization.",
      ros::this_node::getName().c_str(),
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
              .count() *
          1e-6);

  if (param_offline_.enable_log && log_file_pose_.is_open())
  {
    log_file_pose_.close();
  }

  for (const std::shared_ptr<rosbag::Bag>& bag : bags_)
  {
    if (bag->isOpen())
    {
      ROS_INFO("[%s] Closing %s", ros::this_node::getName().c_str(),
               bag->getFileName().c_str());

      bag->close();
    }
  }

  restoreTerminal();

  ros::spin();
}

void SLAMGMappingROS1Offline::pubMap()
{
  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}

void SLAMGMappingROS1Offline::pubPose(const std_msgs::Header& header)
{
  if (!param_offline_.enable_log)
  {
    return;
  }

  GMapping::OrientedPoint mpose =
      gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

  tf2::Quaternion mpose_q;
  mpose_q.setRPY(0, 0, mpose.theta);

  try
  {
    log_file_pose_ << std::fixed << std::setprecision(9) << header.stamp.toSec()
                   << " " << mpose.x << " " << mpose.y << " " << 0 << " "
                   << mpose_q.x() << " " << mpose_q.y() << " " << mpose_q.z()
                   << " " << mpose_q.w() << std::endl;
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "SLAMGMappingROS1Offline::pubPose | error when logging the robot data "
        "(" +
        std::string(e.what()) + ")");
  }
}

void SLAMGMappingROS1Offline::validateAndCreatePath(
    const std::string& file_path)
{
  try
  {
    std::filesystem::path path(file_path);
    std::filesystem::path directory = path.parent_path();
    std::filesystem::path curr_directory = std::filesystem::current_path();

    ROS_INFO("[%s] Current path directory (%s)",
             ros::this_node::getName().c_str(),
             curr_directory.string().c_str());

    if (directory.empty())
    {
      ROS_INFO("[%s] Using current path directory.",
               ros::this_node::getName().c_str());
      return;
    }

    if (std::filesystem::exists(directory))
    {
      if (std::filesystem::is_directory(directory))
      {
        ROS_INFO("[%s] Directory (%s) exists",
                 ros::this_node::getName().c_str(), directory.string().c_str());
        return;
      }
      else
      {
        ROS_ERROR("[%s] Path (%s) exists but is not a directory",
                  ros::this_node::getName().c_str(),
                  directory.string().c_str());

        throw std::runtime_error(
            "SLAMGMappingROS1Offline::validateAndCreatePath | Path (" +
            directory.string() + ") exists but is not a directory");
      }
    }
    else
    {
      ROS_INFO("[%s] Directory doesn't exist. Creating: %s",
               ros::this_node::getName().c_str(), directory.string().c_str());

      if (std::filesystem::create_directory(directory))
      {
        ROS_INFO("[%s] Directory created successfully.",
                 ros::this_node::getName().c_str());
        return;
      }
      else
      {
        ROS_ERROR("[%s] Failed to create directory.",
                  ros::this_node::getName().c_str());

        throw std::runtime_error(
            "SLAMGMappingROS1Offline::validateAndCreatePath | Failed to create "
            "directory (" +
            directory.string() + ")");
      }
    }
  }
  catch (const std::filesystem::filesystem_error& e)
  {
    throw std::runtime_error(
        "SLAMGMappingROS1Offline::validateAndCreatePath | Error resolving path "
        "(" +
        file_path + "): " + e.what());
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "SLAMGMappingROS1Offline::validateAndCreatePath | Error when "
        "processing "
        "path (" +
        file_path + "): " + e.what());
  }
}

void SLAMGMappingROS1Offline::setupTerminal()
{
  if (terminal_modified_)
  {
    return;
  }

  // Save original terminal settings
  const int fd = fileno(stdin);
  tcgetattr(fd, &orig_flags_);

  // Set terminal to raw mode for immediate key detection
  struct termios raw = orig_flags_;
  raw.c_lflag &= ~(ICANON);  // noncanonical mode (input available immediately)
  raw.c_cc[VMIN] = 0;        // polling read mode
  raw.c_cc[VTIME] = 0;       // block if waiting for char

  tcsetattr(fd, TCSANOW, &raw);  // change occur immediately

  // Make stdin non-blocking
  fcntl(fd, F_SETFL, O_NONBLOCK);

  // Hide cursor and clear screen
  std::cout << "\033[2J"  // clear entire screen
            << "\033[H"   // move cursor to home position
            << std::flush;

  terminal_modified_ = true;
}

void SLAMGMappingROS1Offline::restoreTerminal()
{
  if (!terminal_modified_)
  {
    return;
  }

  const int fd = fileno(stdin);
  tcsetattr(fd, TCSANOW, &orig_flags_);

  terminal_modified_ = false;
}

void SLAMGMappingROS1Offline::printTime(const ros::Time& t,
                                        const ros::Duration& duration,
                                        const ros::Duration& bag_length) const
{
  std::cout << std::fixed << std::setprecision(6) << "["
            << (paused_ ? "PAUSED " : "RUNNING") << "] Bag Time: " << t.toSec()
            << "   Duration: " << duration.toSec() << " / "
            << bag_length.toSec() << std::endl;
}

char SLAMGMappingROS1Offline::readTerminalKey() const
{
  char c;

  if (read(STDIN_FILENO, &c, 1) < 0)
  {
    return '\0';
  }

  return c;
}
