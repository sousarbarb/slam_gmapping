#include "slam_gmapping_ros1_offline.h"

#include <fcntl.h>
#include <sys/ioctl.h>

#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ROS
#include <geometry_msgs/TransformStamped.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_msgs/TFMessage.h>

namespace
{

tf2::Transform toTransform(const GMapping::OrientedPoint& p)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, p.theta);
  return tf2::Transform(q, tf2::Vector3(p.x, p.y, 0.0));
}

}  // namespace

SLAMGMappingROS1Offline::SLAMGMappingROS1Offline(const ParamOffline& param)
    : SLAMGMappingROS1API::SLAMGMappingROS1API(),
      param_offline_(param),
      tf2_pub_(std::make_unique<tf2_ros::TransformBroadcaster>())
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

  // Random seed: the base constructor sets seed_ = time(NULL); initMapper()
  // later seeds drand48 with seed_ on the first scan, which makes a run
  // reproducible for a given seed (GMapping draws all samples from drand48)
  if (param_offline_.seed != 0)
  {
    seed_ = param_offline_.seed;
  }

  ROS_INFO("[%s] seed      : %lu%s", ros::this_node::getName().c_str(), seed_,
           (param_offline_.seed != 0) ? "" : " (from time)");

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
    std::string log_file_tf;
    std::string log_file_traj;

    try
    {
      std::filesystem::path log_file_path(param_offline_.log_filename);

      std::filesystem::path dir = log_file_path.parent_path();
      std::string stem = log_file_path.stem().string();
      std::string ext = log_file_path.extension().string();

      log_file_pose = (dir / (stem + "_gmapping_pose" + ext)).string();
      log_file_tf = (dir / (stem + "_gmapping_tf" + ext)).string();
      log_file_traj = (dir / (stem + "_gmapping_traj" + ext)).string();

      ROS_INFO("[%s] log file  : %s", ros::this_node::getName().c_str(),
               log_file_pose.c_str());
      ROS_INFO("[%s] log file  : %s", ros::this_node::getName().c_str(),
               log_file_tf.c_str());
      ROS_INFO("[%s] log file  : %s", ros::this_node::getName().c_str(),
               log_file_traj.c_str());
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

    openLogFile(log_file_pose_, log_file_pose);
    openLogFile(log_file_tf_, log_file_tf);
    openLogFile(log_file_traj_, log_file_traj);
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
  for (std::ofstream* file : {&log_file_pose_, &log_file_tf_, &log_file_traj_})
  {
    if (file->is_open())
    {
      file->close();
    }
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

  // setupTerminal();

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

  std::vector<std::string> tf_static_topics({"/tf_static"});

  rosbag::View full_view;
  rosbag::View tf_static_view;

  for (const std::shared_ptr<rosbag::Bag>& bag : bags_)
  {
    full_view.addQuery(*bag);
    tf_static_view.addQuery(*bag, rosbag::TopicQuery(tf_static_topics));
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

  ROS_INFO("[%s] Loading TF static transforms...",
           ros::this_node::getName().c_str());

  for (rosbag::MessageInstance const& msg : tf_static_view)
  {
    if (msg.instantiate<tf2_msgs::TFMessage>() != nullptr)
    {
      tf2_msgs::TFMessagePtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();

      bool is_static = (msg.getTopic().compare("/tf_static") == 0);

      if (!is_static)
      {
        continue;
      }

      for (const geometry_msgs::TransformStamped& transf : tf_msg->transforms)
      {
        tf2_buffer_.setTransform(transf, ros::this_node::getName(), is_static);

        ROS_INFO("[%s] TF static transform: %s --> %s",
                 ros::this_node::getName().c_str(),
                 transf.header.frame_id.c_str(), transf.child_frame_id.c_str());
      }
    }
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
    /* while (true)
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
    } */

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

  // restoreTerminal();

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

  ROS_INFO(
      "\n\n"
      "[%s] Updating one last time the 2D occupancy grid map...",
      ros::this_node::getName().c_str());

  if (!got_first_scan_)
  {
    throw std::runtime_error(
        "SLAMGMappingROS1Offline::run | no scan was processed (check the scan "
        "topic and the TF tree)");
  }

  updateMap();

  ros::spinOnce();

  if (param_offline_.enable_log && log_file_pose_.is_open())
  {
    writeTrajectory();
  }

  for (std::ofstream* file : {&log_file_pose_, &log_file_tf_, &log_file_traj_})
  {
    if (file->is_open())
    {
      file->close();
    }
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

  // restoreTerminal();

  // --spin false: return, so the node exits (batch runs, required="true")
  if (param_offline_.spin)
  {
    ros::spin();
  }
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

  // pubPose() only runs after initMapper() succeeded, which set the centered
  // laser frame that GMapping estimates
  if (!centered_laser_to_base_ready_)
  {
    initCenteredLaserToBase(header.stamp);
  }

  // Odometry pose of the base at this scan; if unavailable, the scan was
  // skipped by addScan() and no estimate is logged for it
  tf2::Transform odom_to_base;

  try
  {
    const geometry_msgs::TransformStamped tf_msg =
        tf2_buffer_.lookupTransform(odom_frame_, base_frame_, header.stamp);

    const geometry_msgs::Vector3& t = tf_msg.transform.translation;
    const geometry_msgs::Quaternion& q = tf_msg.transform.rotation;

    odom_to_base = tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w),
                                  tf2::Vector3(t.x, t.y, t.z));
  }
  catch (const tf2::TransformException&)
  {
    return;
  }

  const double stamp = header.stamp.toSec();

  // (1) Best particle at every scan. Between updates, processScan() moves
  // every particle by sampling the motion model, so this pose random-walks
  // around odometry until the next update corrects it.
  const GMapping::OrientedPoint mpose =
      gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

  const tf2::Transform map_to_base =
      toTransform(mpose) * centered_laser_to_base_;

  geometry_msgs::TransformStamped msg;

  msg.header.frame_id = map_frame_;
  msg.header.stamp = ros::Time::now();
  msg.child_frame_id = base_frame_;
  msg.transform = tf2::toMsg(map_to_base);

  tf2_pub_->sendTransform(msg);

  writeTUM(log_file_pose_, stamp, map_to_base);

  // (2) map->odom from the last update composed with odom->base: the pose
  // that the standard slam_gmapping node yields through /tf
  map_to_odom_mutex_.lock();
  const tf2::Transform map_to_base_tf = map_to_odom_ * odom_to_base;
  map_to_odom_mutex_.unlock();

  writeTUM(log_file_tf_, stamp, map_to_base_tf);
}

void SLAMGMappingROS1Offline::writeTrajectory()
{
  if (!param_offline_.enable_log || !got_first_scan_ ||
      !centered_laser_to_base_ready_)
  {
    return;
  }

  // (3) Final best particle: its trajectory tree holds one pose per update,
  // consistent with the final map (corrections from resampling included)
  const GMapping::GridSlamProcessor::Particle& best =
      gsp_->getParticles()[gsp_->getBestParticleIndex()];

  std::vector<std::pair<double, GMapping::OrientedPoint>> trajectory;

  for (const GMapping::GridSlamProcessor::TNode* n = best.node; n;
       n = n->parent)
  {
    if (n->reading)
    {
      trajectory.emplace_back(n->reading->getTime(), n->pose);
    }
  }

  for (auto it = trajectory.rbegin(); it != trajectory.rend(); ++it)
  {
    writeTUM(log_file_traj_, it->first,
             toTransform(it->second) * centered_laser_to_base_);
  }

  ROS_INFO("[%s] Final best-particle trajectory: %zu poses",
           ros::this_node::getName().c_str(), trajectory.size());
}

void SLAMGMappingROS1Offline::initCenteredLaserToBase(const ros::Time& stamp)
{
  // centered laser in the laser frame (pose set by initMapper(): laser origin,
  // yaw rotated to the center of the scan, z up even if mounted upside down)
  const geometry_msgs::Pose& cl = centered_laser_pose_.pose;

  const tf2::Transform laser_to_centered_laser(
      tf2::Quaternion(cl.orientation.x, cl.orientation.y, cl.orientation.z,
                      cl.orientation.w),
      tf2::Vector3(cl.position.x, cl.position.y, cl.position.z));

  // base in the laser frame (the same lookup initMapper() just succeeded with)
  tf2::Transform laser_to_base;

  try
  {
    const geometry_msgs::TransformStamped tf_msg =
        tf2_buffer_.lookupTransform(laser_frame_, base_frame_, stamp);

    const geometry_msgs::Vector3& t = tf_msg.transform.translation;
    const geometry_msgs::Quaternion& q = tf_msg.transform.rotation;

    laser_to_base = tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w),
                                   tf2::Vector3(t.x, t.y, t.z));
  }
  catch (const tf2::TransformException& e)
  {
    throw std::runtime_error(
        "SLAMGMappingROS1Offline::initCenteredLaserToBase | unable to look up "
        "the transform from " +
        base_frame_ + " to " + laser_frame_ + " (" + e.what() + ")");
  }

  centered_laser_to_base_ = laser_to_centered_laser.inverse() * laser_to_base;
  centered_laser_to_base_ready_ = true;

  // base_frame_ -> centered laser offset, i.e. the lever arm removed from the
  // logged poses
  const tf2::Transform base_to_cl = centered_laser_to_base_.inverse();
  double roll, pitch, yaw;
  tf2::Matrix3x3(base_to_cl.getRotation()).getRPY(roll, pitch, yaw);

  ROS_INFO(
      "[%s] Logging %s poses; centered laser in %s: x %.4f y %.4f z %.4f (m), "
      "yaw %.3f (deg)",
      ros::this_node::getName().c_str(), base_frame_.c_str(),
      base_frame_.c_str(), base_to_cl.getOrigin().x(),
      base_to_cl.getOrigin().y(), base_to_cl.getOrigin().z(),
      yaw * 180.0 / M_PI);
}

void SLAMGMappingROS1Offline::openLogFile(std::ofstream& file,
                                          const std::string& filename)
{
  file.open(filename);

  if (!file.is_open())
  {
    throw std::runtime_error(
        "SLAMGMappingROS1Offline::openLogFile | error when opening the log "
        "file (" +
        filename + ")");
  }

  file << std::fixed << std::setprecision(9);
}

void SLAMGMappingROS1Offline::writeTUM(std::ofstream& file, double stamp,
                                       const tf2::Transform& pose)
{
  // Planar pose (x, y, yaw; z = 0), as GMapping estimates in 2D; drops the
  // height of base_frame_ below the laser
  double roll, pitch, yaw;
  tf2::Matrix3x3(pose.getRotation()).getRPY(roll, pitch, yaw);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  const tf2::Vector3& t = pose.getOrigin();

  try
  {
    file << stamp << " " << t.x() << " " << t.y() << " " << 0.0 << " " << q.x()
         << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "SLAMGMappingROS1Offline::writeTUM | error when logging the robot "
        "data (" +
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

      if (std::filesystem::create_directories(directory))
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
  orig_flags_.c_lflag |= (ICANON);  // always restore canonical mode
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

  // Filter out control characters and escape sequences
  if (c == '\033')
  {  // ESC character - start of escape sequence
    // Read and discard the rest of the escape sequence
    char temp;
    while (read(STDIN_FILENO, &temp, 1) > 0)
    {
      if (temp >= 'A' && temp <= 'Z') break;  // End of most escape sequences
      if (temp >= 'a' && temp <= 'z') break;
      if (temp == '~') break;  // End of some sequences
    }
    return '\0';
  }

  // Only return printable characters and specific control chars you want
  if ((c >= 32 && c <= 126) || c == '\n' || c == '\r' || c == '\t' || c == 27)
  {
    return c;
  }

  return '\0';  // Ignore other characters
}
