/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/**
/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */
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
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner 
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
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map

Parameters used by GMapping itself:

Laser Parameters:
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)
- @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)
- @b "~/kernelSize" @b [int] search window for the scan matching process
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [int] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)
- @b "~/minimumScore" @b [double] minimum score for considering the outcome of the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

Motion Model Parameters (all standard deviations of a gaussian noise model)
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

- @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

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

// System Header
#include <iostream>
#include <time.h>

// Boost Header
#include <boost/foreach.hpp>

// ROS Header
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

// GMapping Header
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

// Header
#include "map2d_util.h"
#include "slam_gmapping.h"

#define foreach BOOST_FOREACH

SlamGMapping::SlamGMapping() :
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), 
  tf::Point(0, 0, 0 ))),
  m_laser_callback_called_cnt(0), 
  private_nh_("~"), 
  scan_filter_sub_(NULL), 
  scan_filter_(NULL), 
  transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), 
  tf::Point(0, 0, 0 ))),
  m_laser_callback_called_cnt(0),
  node_(nh), 
  private_nh_(pnh), 
  scan_filter_sub_(NULL), 
  scan_filter_(NULL), 
  transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int max_duration_buffer):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), 
  tf::Point(0, 0, 0 ))),
  m_laser_callback_called_cnt(0), 
  private_nh_("~"), 
  scan_filter_sub_(NULL), 
  scan_filter_(NULL), 
  transform_thread_(NULL),
  seed_(seed), 
  tf_(ros::Duration(max_duration_buffer))
{
  init();
}

void SlamGMapping::init()
{

  loadSystemParams();
  loadGridSlamProcParams();
  loadScanMatcherParams();
  loadMotionModelParams();

  m_gsp = new GMapping::GridSlamProcessor();
  ROS_ASSERT(m_gsp);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  m_gsp_laser = NULL;
  m_got_first_scan = false;
  m_got_first_map = false;

  m_xmin = m_params.gridSlamProcParams.x_min;
  m_ymin = m_params.gridSlamProcParams.y_min;
  m_xmax = m_params.gridSlamProcParams.x_max;
  m_ymax = m_params.gridSlamProcParams.y_max;

  map_.map.info.resolution = m_params.gridSlamProcParams.delta;
  map_.map.info.origin.position.x = 0.0;
  map_.map.info.origin.position.y = 0.0;
  map_.map.info.origin.position.z = 0.0;
  map_.map.info.origin.orientation.x = 0.0;
  map_.map.info.origin.orientation.y = 0.0;
  map_.map.info.origin.orientation.z = 0.0;
  map_.map.info.origin.orientation.w = 1.0;

}

void SlamGMapping::loadSystemParams() {

  // Processing Cycle
  private_nh_.param("throttle_scans", m_laser_process_cycle, 1);

  // Map Related Parameter.
  private_nh_.param("occ_thresh", m_occ_thresh, 0.25);
  {
    double tmp;
    private_nh_.param("map_update_interval", tmp, 5.0);
    m_map_update_interval.fromSec(tmp);
  }

  // TF Related Parameter.
  private_nh_.param("base_frame", m_base_frame, std::string("base_link"));
  private_nh_.param("map_frame", m_map_frame, std::string("map"));
  private_nh_.param("odom_frame", m_odom_frame, std::string("odom"));
  private_nh_.param("transform_publish_period", m_transform_publish_period, 0.05);
  private_nh_.param("tf_delay", m_tf_delay, m_transform_publish_period);

}

void SlamGMapping::loadGridSlamProcParams() {

  {
    int tmp_num;
    private_nh_.param("particles", tmp_num, 30);
    m_params.gridSlamProcParams.particle_num = static_cast<unsigned int>(tmp_num);
  }

  // Default Map Size Parameter.
  private_nh_.param("xmin", m_params.gridSlamProcParams.x_min, -100.0);
  private_nh_.param("ymin", m_params.gridSlamProcParams.y_min, -100.0);
  private_nh_.param("xmax", m_params.gridSlamProcParams.x_max, 100.0);
  private_nh_.param("ymax", m_params.gridSlamProcParams.y_max, 100.0);
  private_nh_.param("delta", m_params.gridSlamProcParams.delta, 0.05);

  // Criteria for Registering Scan.
  private_nh_.param("linearUpdate", m_params.gridSlamProcParams.linearThresholdDistance, 1.0);
  private_nh_.param("angularUpdate", m_params.gridSlamProcParams.angularThresholdDistance, 0.5);
  private_nh_.param("temporalUpdate", m_params.gridSlamProcParams.period, -1.0);

  // Resampling Threshold
  private_nh_.param("resampleThreshold", m_params.gridSlamProcParams.resampleThreshold, 0.5);
  private_nh_.param("ogain", m_params.gridSlamProcParams.obsSigmaGain, 3.0);
  private_nh_.param("minimumScore", m_params.gridSlamProcParams.minimumScore, 0.0);

  // Non Modified Parameters?
  private_nh_.param("maxMove", m_params.gridSlamProcParams.maxMove, 1.0);
  private_nh_.param("regScore", m_params.gridSlamProcParams.regScore, 300.0)  ;
  private_nh_.param("critScore", m_params.gridSlamProcParams.critScore, m_params.gridSlamProcParams.regScore * 0.5);

}

void SlamGMapping::loadScanMatcherParams() {

  // Raser Parameters
  private_nh_.param("maxRange", m_params.scanMatcherParams.laserMaxRange, 40.0);
  private_nh_.param("maxUrange", m_params.scanMatcherParams.usableRange, m_params.scanMatcherParams.laserMaxRange);
  private_nh_.param("sigma", m_params.scanMatcherParams.gaussianSigma, 0.05);
  private_nh_.param("lsigma", m_params.scanMatcherParams.likelihoodSigma, 0.075);

  {
    int tmp;
    private_nh_.param("lskip", tmp, 0);  
    m_params.scanMatcherParams.likelihoodSkip = static_cast<unsigned int>(tmp);
  }

  // Optimization Paramter
  private_nh_.param("kernelSize", m_params.scanMatcherParams.kernelSize, 1);
  private_nh_.param("lstep", m_params.scanMatcherParams.optLinearDelta, 0.05);
  private_nh_.param("astep", m_params.scanMatcherParams.optAngularDelta, 0.05);
  private_nh_.param("iterations", m_params.scanMatcherParams.optRecursiveIterations, 5);

  // Scoring Parameter
  private_nh_.param("llsamplerange", m_params.scanMatcherParams.llSamplerange, 0.01);
  private_nh_.param("llsamplestep", m_params.scanMatcherParams.llSampleStep, 0.01);
  private_nh_.param("lasamplerange", m_params.scanMatcherParams.laSampleRange, 0.005);
  private_nh_.param("lasamplestep", m_params.scanMatcherParams.laSampleStep, 0.005);

  // Non Modified Parameters?
  private_nh_.param("enlargeStep", m_params.scanMatcherParams.enlargeStep, 10.0);
  private_nh_.param("fullnessThreshold", m_params.scanMatcherParams.fullnessThreshold, 0.1);
  private_nh_.param("angularOdometryReliability", m_params.scanMatcherParams.angularOdometryReliability, 0.0);
  private_nh_.param("linearOdometryReliability", m_params.scanMatcherParams.linearOdometryReliability, 0.0);
  private_nh_.param("freeCellRatio", m_params.scanMatcherParams.freeCellRatio, std::sqrt(2.0));

  {
    int tmp;
    private_nh_.param("initialBeamSkip", tmp, 0);
    m_params.scanMatcherParams.initialBeamSkip = static_cast<unsigned int>(tmp);
  }

  private_nh_.param("nullLikelihood", m_params.scanMatcherParams.nullLikelihood, -0.5);

}

void SlamGMapping::loadMotionModelParams() {

  private_nh_.param("srr", m_params.motionModelParams.err_ratio_rho_per_rho, 0.1);
  private_nh_.param("srt", m_params.motionModelParams.err_ratio_rho_per_theta, 0.2);
  private_nh_.param("str", m_params.motionModelParams.err_ratio_theta_per_rho, 0.1);
  private_nh_.param("stt", m_params.motionModelParams.err_ratio_theta_per_theta, 0.2);

}

void SlamGMapping::startLiveSlam()
{
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  
  // Result (MAP) Publisher and Service Provider.
  map_publisher_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_meta_publisher_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  m_map_service_server = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  
  // Scan Subscriber Registration.
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, m_odom_frame, 5);
  scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));

  // Prepare publisher thread.
  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, m_transform_publish_period));
}

void SlamGMapping::startReplay(const std::string & bag_fname, std::string scan_topic)
{

  double transform_publish_period;
  ros::NodeHandle private_nh_("~");
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  map_publisher_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_meta_publisher_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  m_map_service_server = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  
  rosbag::Bag bag;
  bag.open(bag_fname, rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string("/tf"));
  topics.push_back(scan_topic);
  rosbag::View viewall(bag, rosbag::TopicQuery(topics));

  // Store up to 5 messages and there error message (if they cannot be processed right away)
  std::queue<std::pair<sensor_msgs::LaserScan::ConstPtr, std::string> > s_queue;
  foreach(rosbag::MessageInstance const m, viewall)
  {
    tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
    if (cur_tf != NULL) {
      for (size_t i = 0; i < cur_tf->transforms.size(); ++i)
      {
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform stampedTf;
        transformStamped = cur_tf->transforms[i];
        tf::transformStampedMsgToTF(transformStamped, stampedTf);
        tf_.setTransform(stampedTf);
      }
    }

    sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
    if (s != NULL) {
      if (!(ros::Time(s->header.stamp)).is_zero())
      {
        s_queue.push(std::make_pair(s, ""));
      }
      // Just like in live processing, only process the latest 5 scans
      if (s_queue.size() > 5) {
        ROS_WARN_STREAM("Dropping old scan: " << s_queue.front().second);
        s_queue.pop();
      }
      // ignoring un-timestamped tf data 
    }

    // Only process a scan if it has tf data
    while (!s_queue.empty())
    {
      try
      {
        tf::StampedTransform t;
        tf_.lookupTransform(s_queue.front().first->header.frame_id, m_odom_frame, s_queue.front().first->header.stamp, t);
        this->laserCallback(s_queue.front().first);
        s_queue.pop();
      }
      // If tf does not have the data yet
      catch(tf2::TransformException& e)
      {
        // Store the error to display it if we cannot process the data after some time
        s_queue.front().second = std::string(e.what());
        break;
      }
    }
  }

  bag.close();
}

void SlamGMapping::publishLoop(double transform_publish_period){

  if(transform_publish_period == 0) {
    return;
  }

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    publishTransform();
    r.sleep();
  }

}

SlamGMapping::~SlamGMapping()
{
  if(transform_thread_){
    transform_thread_->join();
    delete transform_thread_;
  }

  delete m_gsp;
  if(m_gsp_laser) {
    delete m_gsp_laser;
  }
  if (scan_filter_) {
    delete scan_filter_;
  }
  if (scan_filter_sub_) {
    delete scan_filter_sub_;
  }
}

bool
SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  // Get the pose of the centered laser at the right time
  centered_laser_pose_.stamp_ = t;
  // Get the laser's pose that is centered
  tf::Stamped<tf::Transform> odom_pose;
  try {
    tf_.transformPose(m_odom_frame, centered_laser_pose_, odom_pose);
  } catch(tf::TransformException e) {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}

bool
SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan) {

  std::string laser_frame = scan.header.frame_id;
  
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Transform> laser_pose;
  try {
    tf::Stamped<tf::Pose> ident;
    ident.setIdentity();
    ident.frame_id_ = laser_frame;
    ident.stamp_ = scan.header.stamp;
    tf_.transformPose(m_base_frame, ident, laser_pose);
  } catch(tf::TransformException e) {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", e.what());
    return false;
  }

  // create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                                      m_base_frame);
  try {
    tf_.transformPoint(laser_frame, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  } catch(tf::TransformException& e) {
    ROS_WARN("Unable to determine orientation of laser: %s", e.what());
    return false;
  }
  
  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001) {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f", up.z());
    return false;
  }

  double angle_center = (scan.angle_min + scan.angle_max) / 2;
  centered_laser_pose_ = 
    tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center), tf::Vector3(0,0,0)), ros::Time::now(), laser_frame);

  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  m_laser_angles.resize(scan.ranges.size());

  // Make sure angles are started so that they are centered
  double theta = -std::fabs(scan.angle_min - scan.angle_max)/2;
  for(unsigned int i = 0; i < scan.ranges.size(); ++i) {
    m_laser_angles[i] = theta;
    theta += std::fabs(scan.angle_increment);
  }

  ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f"
            , scan.angle_min, scan.angle_max, scan.angle_increment);
  ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f"
            , m_laser_angles.front(), m_laser_angles.back(), std::fabs(scan.angle_increment));

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  m_gsp_beam_cnt = scan.ranges.size();
  m_gsp_laser = new GMapping::RangeSensor("FLASER",
                                         m_gsp_beam_cnt,
                                         fabs(scan.angle_increment),
                                         gmap_pose,
                                         0.0,
                                         m_params.scanMatcherParams.laserMaxRange);
  ROS_ASSERT(m_gsp_laser);

  GMapping::SensorMap smap;
  smap.insert(make_pair(m_gsp_laser->getName(), m_gsp_laser));
  m_gsp->setSensorMap(smap);

  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp)) {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }

  // By default, generateMap is set to false. Set to true only when needed.
  m_gsp->setGenerateMap(false);
  
  m_gsp->init(m_params, initialPose);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1,seed_);

  ROS_INFO("Initialization complete");

  return true;
}

bool
SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
  if(!getOdomPose(gmap_pose, scan.header.stamp)) {
    return false;
  }
  
  if(scan.ranges.size() != m_gsp_beam_cnt) {
    return false;
  }

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  for(unsigned int i=0; i < scan.ranges.size(); i++) {
    // Must filter out short readings, because the mapper won't
    if(scan.ranges[i] < scan.range_min) {
      ranges_double[i] = (double)scan.range_max;
    } else {
      ranges_double[i] = (double)scan.ranges[i];
    }
  }

  // Generating GMapping Range Readering Object.
  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 m_gsp_laser,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);

  ROS_DEBUG("processing scan");

  // Call core gmapping.
  return m_gsp->processScan(reading);
}

void
SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  m_laser_callback_called_cnt++;
  if ((m_laser_callback_called_cnt % m_laser_process_cycle) != 0) {
    return;
  }

  static ros::Time last_map_update(0,0);

  // We can't initialize the mapper until we've got the first scan
  if(!m_got_first_scan) {
    if(!initMapper(*scan)) {
      return;
    }
    m_got_first_scan = true;
  }

  GMapping::OrientedPoint odom_pose;

  bool add_result = addScan(*scan, odom_pose);
  if(add_result) {
    ROS_DEBUG("scan processed");
    GMapping::OrientedPoint mpose = m_gsp->getParticles()[m_gsp->getBestParticleIndex()].pose;

    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

    m_map_to_odom_mutex.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    m_map_to_odom_mutex.unlock();

    // Check map update interval.
    if(!m_got_first_map || (scan->header.stamp - last_map_update) > map_update_interval_) {
      updateMap(*scan);
      last_map_update = scan->header.stamp;
      ROS_DEBUG("Updated the map");
    }
  } else {
    ROS_DEBUG("cannot process scan");
  }
}

double
SlamGMapping::computePoseEntropy() {
  double weight_total=0.0;
  for(std::vector<GMapping::Particle>::const_iterator it = m_gsp->getParticles().begin();
      it != m_gsp->getParticles().end();
      ++it) {
    weight_total += it->weight;
  }

  double entropy = 0.0;
  for(std::vector<GMapping::Particle>::const_iterator it = m_gsp->getParticles().begin();
      it != m_gsp->getParticles().end();
      ++it) {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }

  return -entropy;
}

void
SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{

  boost::mutex::scoped_lock map_lock (m_map_mutex);
  GMapping::ScanMatcher matcher;

  matcher.setMatchingParameters(m_params.scanMatcherParams);
  matcher.setLaserParameters(scan.ranges.size(), &(m_laser_angles[0]), m_gsp_laser->getPose());
  matcher.setGenerateMap(true);

  // Extract the current best partcle.
  GMapping::Particle best = m_gsp->getParticles()[m_gsp->getBestParticleIndex()];
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0) {
    entropy_publisher_.publish(entropy);
  }

  GMapping::Point center;
  center.x = (m_xmin + m_xmax) / 2.0;
  center.y = (m_ymin + m_ymax) / 2.0;

  // Empty Map to be painted.
  GMapping::ScanMatcherMap smap(center, m_xmin, m_ymin, m_xmax, m_ymax, m_params.gridSlamProcParams.delta);

  // Traverse trees that this best particle has. (All scan?)
  for(GMapping::TNode* n = best.node; n; n = n->parent) {

    ROS_DEBUG("  %.3f %.3f %.3f", n->pose.x, n->pose.y, n->pose.theta);
    if(!n->reading) {
      ROS_DEBUG("Reading is NULL");
      continue;
    }

    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    m_xmin = wmin.x; 
    m_ymin = wmin.y;
    m_xmax = wmax.x; 
    m_ymax = wmax.y;
  }

  slam_gmapping::util::convertSmMap2RosMap(smap, m_occ_thresh, map_);
  m_got_first_map = true;

  // Publish Map Information.
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( m_map_frame );
  map_publisher_.publish(map_.map);
  map_meta_publisher_.publish(map_.map.info);
}

bool 
SlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock map_lock (m_map_mutex);
  if(m_got_first_map && map_.map.info.width && map_.map.info.height) {
    res = map_;
    return true;
  } else {
    return false;
  }
}

void SlamGMapping::publishTransform()
{
  m_map_to_odom_mutex.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(m_tf_delay);
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, m_map_frame, m_odom_frame));
  m_map_to_odom_mutex.unlock();
}

