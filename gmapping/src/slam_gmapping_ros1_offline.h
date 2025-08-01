#pragma once

#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

// ROS
#include <rosbag/bag.h>
#include <tf2_ros/message_filter.h>

// Boost
#include <boost/thread.hpp>

// ROS API
#include "slam_gmapping_ros1_api.h"

class SLAMGMappingROS1Offline : public SLAMGMappingROS1API
{
 public:

  struct ParamOffline
  {
    std::vector<std::string> bags;  //!< set of ROS bag files to process
    std::string scan_topic;         //!< scan topic name for 2D laser data
    bool has_duration;         //!< duration from the start time set in options
    double time_start;         //!< start time (s) into the bag files
    double time_duration;      //!< duration (s) to only process from the bags
    bool enable_log;           //!< enable log of robot data (pose) into TUM
    std::string log_filename;  //!< log filename
  };  // struct SLAMGMappingROS1Offline::ParamOffline

 public:

  SLAMGMappingROS1Offline(const ParamOffline& param);
  virtual ~SLAMGMappingROS1Offline();

  void run();

 protected:

  virtual void pubEntropy() final {}
  virtual void pubMap() final;
  virtual void pubPose(const std_msgs::Header& header) final;
  virtual void pubTransform() final {}

  virtual void setupTerminal();
  virtual void restoreTerminal();
  virtual void printTime(const ros::Time& t, const ros::Duration& duration,
                         const ros::Duration& bag_length) const;

  virtual char readTerminalKey() const;

 private:

  SLAMGMappingROS1Offline() = delete;

  void validateAndCreatePath(const std::string& file_path);

 protected:

  ParamOffline param_offline_;

  bool paused_ = false;

  bool terminal_modified_ = false;

  termios orig_flags_;

  ros::Publisher sst_;
  ros::Publisher sstm_;

  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> scan_filter_;

  std::vector<std::shared_ptr<rosbag::Bag>> bags_;

  std::ofstream log_file_pose_;
};  // class SLAMGMappingROS1Offline : public SLAMGMappingROS1API
