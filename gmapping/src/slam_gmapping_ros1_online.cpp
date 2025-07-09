#include "slam_gmapping_ros1_online.h"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h>

SLAMGMappingROS1Online::SLAMGMappingROS1Online()
    : SLAMGMappingROS1API::SLAMGMappingROS1API(),
      tf2_pub_(std::make_unique<tf2_ros::TransformBroadcaster>()),
      tf2_sub_(std::make_unique<tf2_ros::TransformListener>(tf2_buffer_))
{
}

SLAMGMappingROS1Online::SLAMGMappingROS1Online(ros::NodeHandle& nh,
                                               ros::NodeHandle& pnh)
    : SLAMGMappingROS1API::SLAMGMappingROS1API(nh, pnh),
      tf2_pub_(std::make_unique<tf2_ros::TransformBroadcaster>()),
      tf2_sub_(std::make_unique<tf2_ros::TransformListener>(tf2_buffer_))
{
}

SLAMGMappingROS1Online::SLAMGMappingROS1Online(
    long unsigned int seed, long unsigned int max_duration_buffer)
    : SLAMGMappingROS1API::SLAMGMappingROS1API(seed, max_duration_buffer),
      tf2_pub_(nullptr),
      tf2_sub_(nullptr)
{
}

SLAMGMappingROS1Online::~SLAMGMappingROS1Online() {}

void SLAMGMappingROS1Online::startLiveSlam()
{
  entropy_publisher_ =
      nh_priv_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = nh_.advertiseService("dynamic_map",
                             &SLAMGMappingROS1Online::mapCallback, this);

  scan_filter_sub_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>();

  scan_filter_sub_->subscribe(nh_, "scan", 10);

  scan_filter_ =
      std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(
          *scan_filter_sub_, tf2_buffer_, odom_frame_, 10, nh_priv_);

  scan_filter_->registerCallback(&SLAMGMappingROS1API::laserCallback,
                                 static_cast<SLAMGMappingROS1API*>(this));

  transform_thread_ = new boost::thread(boost::bind(
      &SLAMGMappingROS1Online::publishLoop, this, transform_publish_period_));
}

double SLAMGMappingROS1Online::computePoseEntropy()
{
  double weight_total = 0.0;
  for (std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it =
           gsp_->getParticles().begin();
       it != gsp_->getParticles().end(); ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for (std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it =
           gsp_->getParticles().begin();
       it != gsp_->getParticles().end(); ++it)
  {
    if (it->weight / weight_total > 0.0)
      entropy += it->weight / weight_total * log(it->weight / weight_total);
  }
  return -entropy;
}

void SLAMGMappingROS1Online::pubEntropy()
{
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if (entropy.data > 0.0) entropy_publisher_.publish(entropy);
}

void SLAMGMappingROS1Online::pubMap()
{
  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}

void SLAMGMappingROS1Online::pubOdom(const std_msgs::Header&) {}

void SLAMGMappingROS1Online::pubPose(const std_msgs::Header&) {}

void SLAMGMappingROS1Online::pubTransform()
{
  map_to_odom_mutex_.lock();

  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);

  geometry_msgs::TransformStamped msg;

  msg.header.frame_id = map_frame_;
  msg.header.stamp = tf_expiration;
  msg.child_frame_id = odom_frame_;
  msg.transform = tf2::toMsg(map_to_odom_);

  tf2_pub_->sendTransform(msg);

  map_to_odom_mutex_.unlock();
}

bool SLAMGMappingROS1Online::mapCallback(nav_msgs::GetMap::Request& req,
                                         nav_msgs::GetMap::Response& res)
{
  boost::mutex::scoped_lock map_lock(map_mutex_);
  if (got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

void SLAMGMappingROS1Online::publishLoop(double transform_publish_period)
{
  if (transform_publish_period == 0) return;

  ros::Rate r(1.0 / transform_publish_period);
  while (ros::ok())
  {
    pubTransform();
    r.sleep();
  }
}
