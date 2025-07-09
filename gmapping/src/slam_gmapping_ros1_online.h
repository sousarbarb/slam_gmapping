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

#include <iostream>
#include <memory>

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Boost
#include <boost/thread.hpp>

// ROS API
#include "slam_gmapping_ros1_api.h"

class SLAMGMappingROS1Online : public SLAMGMappingROS1API
{
 public:

  SLAMGMappingROS1Online();
  SLAMGMappingROS1Online(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  SLAMGMappingROS1Online(long unsigned int seed,
                         long unsigned int max_duration_buffer);
  virtual ~SLAMGMappingROS1Online();

  void startLiveSlam();

  bool mapCallback(nav_msgs::GetMap::Request& req,
                   nav_msgs::GetMap::Response& res);
  void publishLoop(double transform_publish_period);

 protected:

  double computePoseEntropy();

  virtual void pubEntropy() final;
  virtual void pubMap() final;
  virtual void pubOdom(const std_msgs::Header&) final;
  virtual void pubPose(const std_msgs::Header&) final;
  virtual void pubTransform() final;

 protected:

  ros::Publisher entropy_publisher_;
  ros::Publisher sst_;
  ros::Publisher sstm_;
  ros::ServiceServer ss_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>
      scan_filter_sub_;

  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> scan_filter_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_pub_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_sub_;

  boost::thread* transform_thread_;

};  // class SLAMGMappingROS1Online : public SLAMGMappingROS1API
