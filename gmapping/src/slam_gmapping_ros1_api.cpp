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
/* Originally  Modified by: Charles DuHadway */
/* New Version Modified by: Ricardo B. Sousa */

/**

@mainpage slam_gmapping

@htmlinclude manifest.html

@b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
scans and odometry and computes a map. This map can be
written to a file using e.g.

  "rosrun map_server map_saver static_map:=dynamic_map"

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a
href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a>
: data from a laser range scanner
- @b "/tf": odometry from the robot


Publishes to (name/type):
- @b "/tf"/tf/tfMessage: position relative to the map


@section services
 - @b "~dynamic_map" : returns the map


@section parameters ROS parameters

Reads the following parameters from the parameter server

Parameters used by our GMapping wrapper:

- @b "~throttle_scans": @b [int] throw away every nth laser scan
- @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map
is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two
recalculations of the map


Parameters used by GMapping itself:

Laser Parameters:
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this
range get discarded completely. (default: maximum laser range minus 1 cm, as
received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used
for map building (default: same as maxRange)
- @b "~/sigma" @b [double] standard deviation for the scan matching process
(cell)
- @b "~/kernelSize" @b [int] search window for the scan matching process
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [int] number of refinement steps in the scan matching.
The final "precision" for the match is lstep*2^(-iterations) or
astep*2^(-iterations), respectively.
- @b "~/lsigma" @b [double] standard deviation for the scan matching process
(single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match
(0 = take all rays)
- @b "~/minimumScore" @b [double] minimum score for considering the outcome of
the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces
when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go
up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

Motion Model Parameters (all standard deviations of a gaussian noise model)
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if
the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if
the robot has turned at least this many rads

- @b "~/resampleThreshold" @b [double] threshold at which the particles get
resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle
represents a possible trajectory that the robot has traveled

Likelihood sampling (used in scan matching)
- @b "~/llsamplerange" @b [double] linear range
- @b "~/lasamplerange" @b [double] linear step size
- @b "~/llsamplestep" @b [double] linear range
- @b "~/lasamplestep" @b [double] angular step size

Initial map dimensions and resolution:
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/

#include "slam_gmapping_ros1_api.h"

// ROS
#include <geometry_msgs/TransformStamped.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>

// GMapping C++ Implementation
#include <gmapping/sensor/sensor_odometry/odometrysensor.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SLAMGMappingROS1API::SLAMGMappingROS1API()
    : nh_priv_("~"),
      laser_count_(0),
      tf2_buffer_(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME), false)
{
  seed_ = time(NULL);
  init();
}

SLAMGMappingROS1API::SLAMGMappingROS1API(ros::NodeHandle& nh,
                                         ros::NodeHandle& pnh)
    : nh_(nh),
      nh_priv_(pnh),
      laser_count_(0),
      tf2_buffer_(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME), false)
{
  seed_ = time(NULL);
  init();
}

SLAMGMappingROS1API::SLAMGMappingROS1API(long unsigned int seed,
                                         long unsigned int max_duration_buffer)
    : nh_priv_("~"),
      laser_count_(0),
      tf2_buffer_(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME), false),
      seed_(seed)
{
  init();
}

void SLAMGMappingROS1API::init()
{
  // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  map_to_odom_.setIdentity();

  // The library is pretty chatty
  // gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  gsp_ = new GMapping::GridSlamProcessor();
  ROS_ASSERT(gsp_);

  gsp_laser_ = NULL;
  gsp_odom_ = NULL;

  got_first_scan_ = false;
  got_map_ = false;

  // Parameters used by our GMapping wrapper
  if (!nh_priv_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if (!nh_priv_.getParam("base_frame", base_frame_)) base_frame_ = "base_link";
  if (!nh_priv_.getParam("map_frame", map_frame_)) map_frame_ = "map";
  if (!nh_priv_.getParam("odom_frame", odom_frame_)) odom_frame_ = "odom";

  nh_priv_.param("transform_publish_period", transform_publish_period_, 0.05);

  double tmp;
  if (!nh_priv_.getParam("map_update_interval", tmp)) tmp = 5.0;
  map_update_interval_.fromSec(tmp);

  // Parameters used by GMapping itself
  maxUrange_ = 0.0;
  maxRange_ = 0.0;  // preliminary default, will be set in initMapper()
  if (!nh_priv_.getParam("minimumScore", minimum_score_)) minimum_score_ = 0;
  if (!nh_priv_.getParam("sigma", sigma_)) sigma_ = 0.05;
  if (!nh_priv_.getParam("kernelSize", kernelSize_)) kernelSize_ = 1;
  if (!nh_priv_.getParam("lstep", lstep_)) lstep_ = 0.05;
  if (!nh_priv_.getParam("astep", astep_)) astep_ = 0.05;
  if (!nh_priv_.getParam("iterations", iterations_)) iterations_ = 5;
  if (!nh_priv_.getParam("lsigma", lsigma_)) lsigma_ = 0.075;
  if (!nh_priv_.getParam("ogain", ogain_)) ogain_ = 3.0;
  if (!nh_priv_.getParam("lskip", lskip_)) lskip_ = 0;
  if (!nh_priv_.getParam("srr", srr_)) srr_ = 0.1;
  if (!nh_priv_.getParam("srt", srt_)) srt_ = 0.2;
  if (!nh_priv_.getParam("str", str_)) str_ = 0.1;
  if (!nh_priv_.getParam("stt", stt_)) stt_ = 0.2;
  if (!nh_priv_.getParam("linearUpdate", linearUpdate_)) linearUpdate_ = 1.0;
  if (!nh_priv_.getParam("angularUpdate", angularUpdate_)) angularUpdate_ = 0.5;
  if (!nh_priv_.getParam("temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;
  if (!nh_priv_.getParam("resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;
  if (!nh_priv_.getParam("particles", particles_)) particles_ = 30;
  if (!nh_priv_.getParam("xmin", xmin_)) xmin_ = -100.0;
  if (!nh_priv_.getParam("ymin", ymin_)) ymin_ = -100.0;
  if (!nh_priv_.getParam("xmax", xmax_)) xmax_ = 100.0;
  if (!nh_priv_.getParam("ymax", ymax_)) ymax_ = 100.0;
  if (!nh_priv_.getParam("delta", delta_)) delta_ = 0.05;
  if (!nh_priv_.getParam("occ_thresh", occ_thresh_)) occ_thresh_ = 0.25;
  if (!nh_priv_.getParam("llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;
  if (!nh_priv_.getParam("llsamplestep", llsamplestep_)) llsamplestep_ = 0.01;
  if (!nh_priv_.getParam("lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;
  if (!nh_priv_.getParam("lasamplestep", lasamplestep_)) lasamplestep_ = 0.005;

  if (!nh_priv_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;
}

SLAMGMappingROS1API::~SLAMGMappingROS1API()
{
  delete gsp_;
  if (gsp_laser_) delete gsp_laser_;
  if (gsp_odom_) delete gsp_odom_;
}

bool SLAMGMappingROS1API::getOdomPose(GMapping::OrientedPoint& gmap_pose,
                                      const ros::Time& t)
{
  // Get the pose of the centered laser at the right time
  centered_laser_pose_.header.stamp = t;
  // Get the laser's pose that is centered
  geometry_msgs::PoseStamped odom_pose;
  try
  {
    tf2_buffer_.transform(centered_laser_pose_, odom_pose, odom_frame_);
  }
  catch (const tf2::TransformException& e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf2::getYaw(odom_pose.pose.orientation);

  gmap_pose = GMapping::OrientedPoint(odom_pose.pose.position.x,
                                      odom_pose.pose.position.y, yaw);
  return true;
}

bool SLAMGMappingROS1API::initMapper(const sensor_msgs::LaserScan& scan)
{
  laser_frame_ = scan.header.frame_id;

  // create a point 1m above the laser position and transform it into the
  // laser-frame
  geometry_msgs::Vector3Stamped up_old;
  up_old.vector.x = 0;
  up_old.vector.y = 0;
  up_old.vector.z = 1;
  up_old.header.frame_id = base_frame_;
  up_old.header.stamp = scan.header.stamp;
  geometry_msgs::Vector3Stamped up;

  try
  {
    tf2_buffer_.transform(up_old, up, laser_frame_);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.vector.z);
  }
  catch (const tf2::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s", e.what());
    return false;
  }

  // gmapping doesnt take roll or pitch into account. So check for correct
  // sensor alignment.
  if (fabs(fabs(up.vector.z) - 1) > 0.001)
  {
    ROS_WARN(
        "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but "
        "gave: %.5f",
        up.vector.z);
    return false;
  }

  gsp_laser_beam_count_ = scan.ranges.size();

  double angle_center = (scan.angle_min + scan.angle_max) / 2;

  tf2::Quaternion centered_laser_pose_q;

  if (up.vector.z > 0)
  {
    centered_laser_pose_q.setRPY(0, 0, angle_center);

    do_reverse_range_ = scan.angle_min > scan.angle_max;
    centered_laser_pose_.header.frame_id = laser_frame_;
    centered_laser_pose_.header.stamp = scan.header.stamp;
    centered_laser_pose_.pose.position.x = 0;
    centered_laser_pose_.pose.position.y = 0;
    centered_laser_pose_.pose.position.z = 0;
    centered_laser_pose_.pose.orientation = tf2::toMsg(centered_laser_pose_q);
    ROS_INFO("Laser is mounted upwards.");
  }
  else
  {
    centered_laser_pose_q.setRPY(M_PI, 0, -angle_center);

    do_reverse_range_ = scan.angle_min < scan.angle_max;
    centered_laser_pose_.header.frame_id = laser_frame_;
    centered_laser_pose_.header.stamp = scan.header.stamp;
    centered_laser_pose_.pose.position.x = 0;
    centered_laser_pose_.pose.position.y = 0;
    centered_laser_pose_.pose.position.z = 0;
    centered_laser_pose_.pose.orientation = tf2::toMsg(centered_laser_pose_q);
    ROS_INFO("Laser is mounted upside down.");
  }

  // Compute the angles of the laser from -x to x, basically symmetric and in
  // increasing order
  laser_angles_.resize(scan.ranges.size());
  // Make sure angles are started so that they are centered
  double theta = -std::fabs(scan.angle_min - scan.angle_max) / 2;
  for (unsigned int i = 0; i < scan.ranges.size(); ++i)
  {
    laser_angles_[i] = theta;
    theta += std::fabs(scan.angle_increment);
  }

  ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f",
            scan.angle_min, scan.angle_max, scan.angle_increment);
  ROS_DEBUG(
      "Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: "
      "%.3f",
      laser_angles_.front(), laser_angles_.back(),
      std::fabs(scan.angle_increment));

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // setting maxRange and maxUrange here so we can set a reasonable default
  if (!nh_priv_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;
  if (!nh_priv_.getParam("maxUrange", maxUrange_)) maxUrange_ = maxRange_;

  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  gsp_laser_ = new GMapping::RangeSensor("FLASER", gsp_laser_beam_count_,
                                         fabs(scan.angle_increment), gmap_pose,
                                         0.0, maxRange_);
  ROS_ASSERT(gsp_laser_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);

  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;
  if (!getOdomPose(initialPose, scan.header.stamp))
  {
    ROS_WARN(
        "Unable to determine inital pose of laser! Starting point will be set "
        "to zero.");
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }

  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_, kernelSize_,
                              lstep_, astep_, iterations_, lsigma_, ogain_,
                              lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_, delta_,
                                initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1, seed_);

  ROS_INFO("Initialization complete");

  return true;
}

bool SLAMGMappingROS1API::addScan(const sensor_msgs::LaserScan& scan,
                                  GMapping::OrientedPoint& gmap_pose)
{
  if (!getOdomPose(gmap_pose, scan.header.stamp)) return false;

  if (scan.ranges.size() != gsp_laser_beam_count_) return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the
  // readings.
  if (do_reverse_range_)
  {
    ROS_DEBUG("Inverting scan");
    int num_ranges = scan.ranges.size();
    for (int i = 0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if (scan.ranges[num_ranges - i - 1] < scan.range_min)
        ranges_double[i] = (double) scan.range_max;
      else
        ranges_double[i] = (double) scan.ranges[num_ranges - i - 1];
    }
  }
  else
  {
    for (unsigned int i = 0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if (scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double) scan.range_max;
      else
        ranges_double[i] = (double) scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(), ranges_double, gsp_laser_,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);

  /*
  ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
            scan.header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */
  ROS_DEBUG("processing scan");

  return gsp_->processScan(reading);
}

void SLAMGMappingROS1API::laserCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0) return;

  static ros::Time last_map_update(0, 0);

  // We can't initialize the mapper until we've got the first scan
  if (!got_first_scan_)
  {
    if (!initMapper(*scan)) return;
    got_first_scan_ = true;
  }

  GMapping::OrientedPoint odom_pose;

  if (addScan(*scan, odom_pose))
  {
    ROS_DEBUG("scan processed");

    GMapping::OrientedPoint mpose =
        gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y,
              odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x,
              mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf2::Quaternion laser_to_map_q;
    laser_to_map_q.setRPY(0, 0, mpose.theta);

    tf2::Transform laser_to_map =
        tf2::Transform(laser_to_map_q, tf2::Vector3(mpose.x, mpose.y, 0.0))
            .inverse();

    tf2::Quaternion odom_to_laser_q;
    odom_to_laser_q.setRPY(0, 0, odom_pose.theta);

    tf2::Transform odom_to_laser = tf2::Transform(
        odom_to_laser_q, tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    map_to_odom_mutex_.unlock();

    if (!got_map_ ||
        (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      updateMap(*scan);
      last_map_update = scan->header.stamp;
      ROS_DEBUG("Updated the map");
    }
  }
  else
    ROS_DEBUG("cannot process scan");

  pubPose(scan->header);
}

void SLAMGMappingROS1API::updateMap(const sensor_msgs::LaserScan& scan)
{
  ROS_DEBUG("Update map");
  boost::mutex::scoped_lock map_lock(map_mutex_);
  GMapping::ScanMatcher matcher;

  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
      gsp_->getParticles()[gsp_->getBestParticleIndex()];

  pubEntropy();

  if (!got_map_)
  {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  GMapping::Point center;
  center.x = (xmin_ + xmax_) / 2.0;
  center.y = (ymin_ + ymax_) / 2.0;

  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, delta_);

  ROS_DEBUG("Trajectory tree:");
  for (GMapping::GridSlamProcessor::TNode* n = best.node; n; n = n->parent)
  {
    ROS_DEBUG("  %.3f %.3f %.3f", n->pose.x, n->pose.y, n->pose.theta);
    if (!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  // the map may have expanded, so resize ros message as well
  if (map_.map.info.width != (unsigned int) smap.getMapSizeX() ||
      map_.map.info.height != (unsigned int) smap.getMapSizeY())
  {
    // NOTE: The results of ScanMatcherMap::getSize() are different from the
    // parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(
        GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x;
    ymin_ = wmin.y;
    xmax_ = wmax.x;
    ymax_ = wmax.y;

    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)",
              smap.getMapSizeX(), smap.getMapSizeY(), xmin_, ymin_, xmax_,
              ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x,
              map_.map.info.origin.position.y);
  }

  for (int x = 0; x < smap.getMapSizeX(); x++)
  {
    for (int y = 0; y < smap.getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ = smap.cell(p);
      assert(occ <= 1.0);
      if (occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if (occ > occ_thresh_)
      {
        // map_.map.data[MAP_IDX(map_.map.info.width, x, y)] =
        // (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  // make sure to set the header information on the map
  map_.map.header.stamp = scan.header.stamp;
  map_.map.header.frame_id = map_frame_;

  pubMap();
}
