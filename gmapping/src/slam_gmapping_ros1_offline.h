#pragma once

#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

// ROS
#include <rosbag/bag.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>

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
    bool has_duration = false;      //!< duration from the start time set
    double time_start = 0.0;        //!< start time (s) into the bag files
    double time_duration = 0.0;     //!< duration (s) to only process from bags
    bool enable_log = false;   //!< enable log of robot data (pose) into TUM
    std::string log_filename;  //!< log filename
    unsigned long seed = 0;    //!< GMapping RNG seed (0: from time)
    bool spin = true;          //!< keep spinning after processing the bags
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
  void openLogFile(std::ofstream& file, const std::string& filename);
  void writeTUM(std::ofstream& file, double stamp, const tf2::Transform& pose);
  void writeTrajectory();
  void initCenteredLaserToBase(const ros::Time& stamp);

 protected:

  ParamOffline param_offline_;

  bool paused_ = false;

  bool terminal_modified_ = false;

  termios orig_flags_;

  ros::Publisher sst_;
  ros::Publisher sstm_;

  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> scan_filter_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_pub_;

  std::vector<std::shared_ptr<rosbag::Bag>> bags_;

  // Logs (TUM) of the base_frame_ pose in map_frame_
  std::ofstream log_file_pose_;  //!< best particle, every scan
  std::ofstream log_file_tf_;    //!< map->odom (last update) * odom, every scan
  std::ofstream log_file_traj_;  //!< final best particle, one pose per update

  //! GMapping estimates the pose of the centered laser (see initMapper());
  //! this static transform maps it to base_frame_
  tf2::Transform centered_laser_to_base_;
  bool centered_laser_to_base_ready_ = false;
};  // class SLAMGMappingROS1Offline : public SLAMGMappingROS1API
