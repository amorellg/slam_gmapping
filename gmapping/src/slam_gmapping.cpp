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
- @b "~/kernelSize" @b [double] search window for the scan matching process
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [double] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)

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
- @b "~/lasamplestep" @b [double] angular setp size

Initial map dimensions and resolution:
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/



#include "slam_gmapping.h"

#include <iostream>

#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), transform_thread_(NULL)
{
  // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  got_map_ = false;

  ros::NodeHandle private_nh_("~");

  // Parameters used by our GMapping wrapper
  private_nh_.param("map_update_interval", map_update_interval, 5.0);
  private_nh_.param("throttle_scans", throttle_scans_, 1);
  private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
  private_nh_.param<std::string>("map_frame",  map_frame_,  "map");
  private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
  private_nh_.param("transform_publish_period", transform_publish_period_, .05);
  private_nh_.param("particles_publish_period", transform_publish_period_, .01);
  private_nh_.param("rng_seed", rng_seed_, int(time(NULL)));

  // Parameters used by GMapping itself
  private_nh_.param("sigma", sigma_, 0.05);
  private_nh_.param("kernelSize", kernelSize_, 1);
  private_nh_.param("lstep", lstep_, 0.05);
  private_nh_.param("astep", astep_, 0.05);
  private_nh_.param("iterations", iterations_, 5);
  private_nh_.param("lsigma", lsigma_, 0.075);
  private_nh_.param("ogain", ogain_, 3.0);
  private_nh_.param("lskip", lskip_, 0);
  private_nh_.param("tf_delay", tf_delay_, transform_publish_period_);
  private_nh_.param("srr", srr_, 0.1);
  private_nh_.param("srt", srr_, 0.2);
  private_nh_.param("str", srr_, 0.1);
  private_nh_.param("stt", srr_, 0.2);
  private_nh_.param("linearUpdate", linearUpdate_, 1.0);
  private_nh_.param("angularUpdate", angularUpdate_, 0.5);
  private_nh_.param("temporalUpdate", temporalUpdate_, -1.0);
  private_nh_.param("resampleThreshold", resampleThreshold_, 0.5);
  private_nh_.param("particles", particles_, 30);
  private_nh_.param("xmin", xmin_, -100.0);
  private_nh_.param("ymin", ymin_, -100.0);
  private_nh_.param("xmax", xmax_, 100.0);
  private_nh_.param("ymax", ymax_, 100.0);
  private_nh_.param("delta", delta_, 0.05);
  private_nh_.param("occ_thresh", occ_thresh_, 0.25);
  private_nh_.param("llsamplerange", llsamplerange_, 0.01);
  private_nh_.param("llsamplestep", llsamplestep_, 0.01);
  private_nh_.param("lasamplerange", lasamplerange_, 0.005);
  private_nh_.param("lasamplestep", lasamplestep_, 0.005);
  ROS_ASSERT_MSG(private_nh_.getParam("maxRange", maxRange_),
                 "Range sensor maxRange parameter is required");
  private_nh_.param("maxUrange", maxUrange_, maxRange_);
  //Parameters to be implemented likely...
  //private_nh_.param("enlargeStep", enlargeStep_, 10.0);
  //private_nh_.param("fullnessThreshold", fullnessThreshold_, 0.1);
  //private_nh_.param("angularOdometryReliability", angularOdometryReliability_, 0.);
  //private_nh_.param("linearOdometryReliability",  linearOdometryReliability_,  0.);
  //private_nh_.param("freeCellRatio", freeCellRatio_, sqrt(2.));
  //private_nh_.param("freeSpaceWeight", freesw_, 0.);
  //private_nh_.param("IMU_yaw_default_cov",     IMU_yaw_default_cov, 1.4);
  //private_nh_.param("GPS_available",           GPS_available,       false);
  //private_nh_.param("GPS_initialization_wait", GPS_init_wait,       10.0);
  //private_nh_.param("initialBeamsSkip", initialBeamsSkip_, 0);

  srand48(rng_seed_); // Seed for the desired sequence of random numbers
  map_update_interval_.fromSec(map_update_interval);
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));

  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));

  // The library is pretty chatty
  //gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  gsp_ = new GMapping::GridSlamProcessor(&node_, map_frame_);
  ROS_ASSERT(gsp_);

//  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
//  ROS_ASSERT(gsp_odom_);

//INITIALPOSE NO MOLA AQUI
////ros::Time wait_start = ros::Time::now();
////ROS_WARN("entro");
////while((ros::Time::now() - wait_start) < ros::Duration(10))
////{
////    if(tf_.waitForTransform(odom_frame_, base_frame_,
////                            ros::Time::now() , ros::Duration(0.2)))
////    {
////      break;
////    }
////}
////ROS_WARN("salgo");
////scan.header.stamp = ros::Time::now();
//  if(!getOdomPose(initialPose, scan.header.stamp, scan.header.frame_id))
//  //if(!getOdomPose(initialPose, ros::Time().now(), base_frame_))
//  {
//    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
//    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
//  }

  //FIXME initialPose con GPS si hay
  GMapping::OrientedPoint initialPose;
  initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);

//  if (!gsp_->m_isInit)
//  {
//SACAR MAXURANGE Y MAXRANGE DE CADA LASER ????
  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  //gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);

  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  ROS_INFO("Initialization complete");
//  }


}

void SlamGMapping::publishLoop(double transform_publish_period)
{
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    publishTransform();
    r.sleep();
  }
}

SlamGMapping::~SlamGMapping()
{
  if(transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
  for (GMapping::SensorMap::iterator it = gsp_->m_sensors.begin(); it != gsp_->m_sensors.end(); it++)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Deleting sensor " << it->first << std::endl;
    delete it->second;
  }
  delete gsp_;
//  if(gsp_odom_)
//    delete gsp_odom_;
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

bool
SlamGMapping::getOdomPose( GMapping::OrientedPoint& gmap_pose
                         , const ros::Time& t
                         , const std::string& f
                         )
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident ( tf::Transform( tf::createQuaternionFromRPY(0,0,0)
                                             , tf::Vector3(0,0,0)
                                             )
                              , t
                              , f
                              );
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
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
SlamGMapping::addRangeSensor(const sensor_msgs::LaserScan& scan)
{
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = scan.header.frame_id;
  ident.stamp_ = scan.header.stamp;
  try
  {
    tf_.transformPose(base_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

//  double yaw = tf::getYaw(laser_pose.getRotation());
//
//  GMapping::OrientedPoint gmap_pose(laser_pose.getOrigin().x(),
//                                    laser_pose.getOrigin().y(),
//                                    yaw);
//  ROS_DEBUG("laser's pose wrt base: %.3f %.3f %.3f",
//            laser_pose.getOrigin().x(),
//            laser_pose.getOrigin().y(),
//            yaw);

  // create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                                      base_frame_);
  try
  {
    tf_.transformPoint(scan.header.frame_id, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN( "Unable to determine orientation of range sensor %s : %s"
            , scan.header.frame_id.c_str()
            , e.what());
    return false;
  }

  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN( "Range sensor %s has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f"
            , scan.header.frame_id.c_str()
            , up.z());
    return false;
  }

  int orientationFactor;
  if (up.z() > 0)
  {
    orientationFactor = 1;
    ROS_INFO("Range sensor %s is mounted upwards.",scan.header.frame_id.c_str());
  }
  else
  {
    orientationFactor = -1;
    ROS_INFO("Range sensor %s is mounted upside down.",scan.header.frame_id.c_str());
  }

  ROS_DEBUG("Laser angles top down in laser-frame: min: %.3f max: %.3f inc: %.3f"
           ,orientationFactor * scan.angle_min
           ,orientationFactor * scan.angle_max
           ,orientationFactor * scan.angle_increment);

//  GMapping::OrientedPoint gmap_pose(0, 0, 0);
  // Laser pose wrt base link
  GMapping::OrientedPoint gmap_pose( laser_pose.getOrigin().x()
                                   , laser_pose.getOrigin().y()
                                   , tf::getYaw(laser_pose.getRotation())
                                   );
  ROS_DEBUG( "Range sensor %s pose wrt base: x= %.3f y= %.3f theta= %.3f."
           , scan.header.frame_id.c_str()
           , laser_pose.getOrigin().x()
           , laser_pose.getOrigin().y()
           , tf::getYaw(laser_pose.getRotation())
           );

  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.

  GMapping::RangeSensor*
  gsp_laser_ = new GMapping::RangeSensor(scan.header.frame_id,
                                         scan.ranges.size(),
                                         scan.angle_increment,
                                         orientationFactor,
                                         gmap_pose,
                                         0.0,
                                         maxRange_);
  ROS_ASSERT(gsp_laser_);

  gsp_->m_sensors.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
//  gsp_->setSensorMap(gsp_->m_sensors, scan.header.frame_id);

  return true;
}

bool
SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
  // Read rangeSensor from the SensorMap
  GMapping::SensorMap::const_iterator laser_it=gsp_->m_sensors.find(scan.header.frame_id);
  ROS_ASSERT(laser_it!=gsp_->m_sensors.end());
  const GMapping::RangeSensor* rangeSensor=dynamic_cast<const GMapping::RangeSensor*>((laser_it->second));
  ROS_ASSERT(rangeSensor);

  //if(!getOdomPose(gmap_pose, scan.header.stamp, scan.header.frame_id))
  if(!getOdomPose(gmap_pose, scan.header.stamp, base_frame_))
     return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  if (rangeSensor->getOrientation() < 0)
  {
    ROS_DEBUG("Inverting scan");
    int num_ranges = scan.ranges.size();
    for(int i=0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  }
  else
  {
    for(unsigned int i=0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 rangeSensor,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);
  //reading.setPose(rangeSensor->getPose());

  /*
  ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
            scan.header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */

  return gsp_->processScan(reading);
}

void
SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // We can't initialize the mapper until we've got the first scan
  if (gsp_->m_sensors.find(scan->header.frame_id) == gsp_->m_sensors.end())
    if(!addRangeSensor(*scan))
      return;

  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0,0);

  GMapping::OrientedPoint odom_pose;
  if(addScan(*scan, odom_pose))
  {
    ROS_DEBUG("scan processed");

    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    map_to_odom_mutex_.unlock();

    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      updateMap(*scan);
      last_map_update = scan->header.stamp;
      ROS_DEBUG("Updated the map");
    }
  }
}

double
SlamGMapping::computePoseEntropy()
{
  double weight_total=0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}

void
SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
  boost::mutex::scoped_lock(map_mutex_);

  // Read rangeSensor from the SensorMap
  GMapping::SensorMap::const_iterator laser_it=gsp_->m_sensors.find(scan.header.frame_id);
  ROS_ASSERT(laser_it!=gsp_->m_sensors.end());
  const GMapping::RangeSensor* rangeSensor=dynamic_cast<const GMapping::RangeSensor*>((laser_it->second));
  ROS_ASSERT(rangeSensor);

  GMapping::ScanMatcher matcher;
//  matcher.setLaserParameters(scan.ranges.size()
//                            ,rangeSensor->getAngles()
//                            ,rangeSensor->getPose());
//  matcher.setlaserMaxRange(maxRange_);
//  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);

  if(!got_map_) {
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
  center.x=(xmin_ + xmax_) / 2.0;
  center.y=(ymin_ + ymax_) / 2.0;

  GMapping::ScanMatcherMap smmap(center, xmin_, ymin_, xmax_, ymax_,
                                delta_);

  ROS_DEBUG("Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    ROS_DEBUG("  %.3f %.3f %.3f",
              n->pose.x,
              n->pose.y,
              n->pose.theta);
    if(!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher.setlaserMaxRange((*rangeSensor).getMaxRange());
    matcher.setusableRange((*rangeSensor).getMaxRange()); //maxUrange_
    matcher.invalidateActiveArea();
    //matcher.computeActiveArea(smmap, n->pose, &((*n->reading)[0]));
    //matcher.registerScan(smmap, n->pose, &((*n->reading)[0]));
    matcher.computeActiveArea(smmap, n->pose, *(n->reading));
    matcher.registerScan(smmap, n->pose, *(n->reading));
  }

  // the map may have expanded, so resize ros message as well
  if(map_.map.info.width != (unsigned int) smmap.getMapSizeX() || map_.map.info.height != (unsigned int) smmap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smmap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smmap.map2world(GMapping::IntPoint(smmap.getMapSizeX(), smmap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;

    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smmap.getMapSizeX(), smmap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smmap.getMapSizeX();
    map_.map.info.height = smmap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  for(int x=0; x < smmap.getMapSizeX(); x++)
  {
    for(int y=0; y < smmap.getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ=smmap.cell(p);
      ROS_ASSERT(occ <= 1.0);
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ > occ_thresh_)
      {
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}

bool
SlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock(map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}
