/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the
 * names of its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Brian Gerkey */
/* Modified by: Ricardo B. Sousa */

#pragma once

#include <vector>

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

// GMapping C++ Implementation
#include <gmapping/gridfastslam/gridslamprocessor.h>
#include <gmapping/sensor/sensor_base/sensor.h>

// Boost
#include <boost/thread.hpp>

class SLAMGMappingROS1API
{
 public:

  SLAMGMappingROS1API();
  SLAMGMappingROS1API(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  SLAMGMappingROS1API(long unsigned int seed,
                      long unsigned int max_duration_buffer);
  virtual ~SLAMGMappingROS1API();

  void init();
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

 protected:

  bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);
  bool initMapper(const sensor_msgs::LaserScan& scan);
  bool addScan(const sensor_msgs::LaserScan& scan,
               GMapping::OrientedPoint& gmap_pose);
  void updateMap(const sensor_msgs::LaserScan& scan);

  virtual void pubEntropy() = 0;
  virtual void pubMap() = 0;
  virtual void pubPose(const std_msgs::Header&) = 0;
  virtual void pubTransform() = 0;

 protected:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  tf2_ros::Buffer tf2_buffer_;

  GMapping::GridSlamProcessor* gsp_;
  GMapping::RangeSensor* gsp_laser_;
  // The angles in the laser, going from -x to x (adjustment is made to get the
  // laser between symmetrical bounds as that's what gmapping expects)
  std::vector<double> laser_angles_;
  // The pose, in the original laser frame, of the corresponding centered laser
  // with z facing up
  geometry_msgs::PoseStamped centered_laser_pose_;
  // Depending on the order of the elements in the scan and the orientation of
  // the scan frame, We might need to change the order of the scan
  bool do_reverse_range_;
  unsigned int gsp_laser_beam_count_;
  GMapping::OdometrySensor* gsp_odom_;

  bool got_first_scan_;
  bool got_map_;

  nav_msgs::GetMap::Response map_;
  ros::Duration map_update_interval_;
  tf2::Transform map_to_odom_;
  boost::mutex map_to_odom_mutex_;
  boost::mutex map_mutex_;

  int laser_count_;
  int throttle_scans_;

  std::string base_frame_;
  std::string laser_frame_;
  std::string map_frame_;
  std::string odom_frame_;

  // Parameters used by GMapping
  double maxRange_;
  double maxUrange_;
  double maxrange_;
  double minimum_score_;
  double sigma_;
  int kernelSize_;
  double lstep_;
  double astep_;
  int iterations_;
  double lsigma_;
  double ogain_;
  int lskip_;
  double srr_;
  double srt_;
  double str_;
  double stt_;
  double linearUpdate_;
  double angularUpdate_;
  double temporalUpdate_;
  double resampleThreshold_;
  int particles_;
  double xmin_;
  double ymin_;
  double xmax_;
  double ymax_;
  double delta_;
  double occ_thresh_;
  double llsamplerange_;
  double llsamplestep_;
  double lasamplerange_;
  double lasamplestep_;

  unsigned long int seed_;

  double transform_publish_period_;
  double tf_delay_;

};  // class SLAMGMappingROS1API
