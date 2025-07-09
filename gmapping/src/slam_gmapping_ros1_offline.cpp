#include "slam_gmapping_ros1_offline.h"

#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>

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
  ROS_INFO("[%s] odom topic: %s", ros::this_node::getName().c_str(),
           param_offline_.odom_topic.c_str());
  ROS_INFO("[%s] scan topic: %s", ros::this_node::getName().c_str(),
           param_offline_.scan_topic.c_str());
  ROS_INFO("[%s] log file  : %s", ros::this_node::getName().c_str(),
           param_offline_.enable_log ? param_offline_.log_filename.c_str()
                                     : "not enabled");
  ROS_INFO("[%s] log file  : %s", ros::this_node::getName().c_str(),
           param_offline_.enable_log ? param_offline_.log_filename.c_str()
                                     : "not enabled");

  // Log file processing
  if (param_offline_.enable_log)
  {
    if (param_offline_.log_filename.empty())
    {
      throw std::runtime_error(
          "SLAMGMappingROS1Offline::SLAMGMappingROS1Offline | empty filename "
          "when log enabled");
    }

    std::string log_file_odom;
    std::string log_file_pose;

    try
    {
      std::filesystem::path log_file_path(param_offline_.log_filename);

      log_file_odom = log_file_path.stem().string() + "_odom" +
                      log_file_path.extension().string();
      log_file_pose = log_file_path.stem().string() + "_pose" +
                      log_file_path.extension().string();
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

    validateAndCreatePath(log_file_odom);
    validateAndCreatePath(log_file_pose);

    try
    {
      log_file_odom_ = std::ofstream(log_file_odom);

      if (!log_file_odom_.is_open())
      {
        throw std::runtime_error(
            "SLAMGMappingROS1Offline::SLAMGMappingROS1Offline | file (" +
            param_offline_.log_filename + ") for odometry data not opened");
      }

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
          "opening the log "
          "files (" +
          log_file_odom + " ; " + log_file_pose + "): " + e.what());
    }
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
  if (param_offline_.enable_log && log_file_odom_.is_open())
  {
    log_file_odom_.close();
  }

  if (param_offline_.enable_log && log_file_pose_.is_open())
  {
    log_file_pose_.close();
  }

  for (const std::shared_ptr<rosbag::Bag>& bag : bags_)
  {
    ROS_INFO("[%s] Closing %s", ros::this_node::getName().c_str(),
             bag->getFileName().c_str());

    bag->close();
  }
}

void SLAMGMappingROS1Offline::run()
{
  std::cout << std::endl;

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

  ROS_INFO("[%s] Start  time (s): %.9lf", ros::this_node::getName().c_str(),
           initial_time.toSec());

  if (param_offline_.has_duration)
  {
    finish_time = initial_time + ros::Duration(param_offline_.time_duration);

    ROS_INFO("[%s] Finish time (s): %.9lf", ros::this_node::getName().c_str(),
             finish_time.toSec());
    ROS_INFO("[%s] Total  time (s): %.9lf\n", ros::this_node::getName().c_str(),
             ros::Duration(finish_time - initial_time).toSec());
  }
  else
  {
    ROS_INFO("[%s] Finish time (s): %.9lf (end of the bags)",
             ros::this_node::getName().c_str(), full_view.getEndTime().toSec());
    ROS_INFO("[%s] Total  time (s): %.9lf\n", ros::this_node::getName().c_str(),
             ros::Duration(full_view.getEndTime() - initial_time).toSec());
  }

  rosbag::View view;

  for (const std::shared_ptr<rosbag::Bag>& bag : bags_)
  {
    view.addQuery(*bag, initial_time, finish_time);
  }

  for (rosbag::MessageInstance const& msg : view)
  {
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
}

void SLAMGMappingROS1Offline::pubMap()
{
  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}

void SLAMGMappingROS1Offline::pubOdom(const std_msgs::Header&) {}

void SLAMGMappingROS1Offline::pubPose(const std_msgs::Header&) {}

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
