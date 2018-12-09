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

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"

#include <boost/thread.hpp>

class SlamGMapping
{
  public:
    SlamGMapping();
    SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    SlamGMapping(unsigned long int seed, unsigned long int max_duration_buffer);
    ~SlamGMapping();

    void init();
    void startLiveSlam();
    void startReplay(const std::string & bag_fname, std::string scan_topic);

  private:
    void publishTransform();
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);
    void publishLoop(double transform_publish_period);
    void updateMap(const sensor_msgs::LaserScan& scan);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);
    bool initMapper(const sensor_msgs::LaserScan& scan);
    bool addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose);
    double computePoseEntropy();

    void loadSystemParams();
    void loadGridSlamProcParams();
    void loadScanMatcherParams();
    void loadMotionModelParams();

  private:
    // Node Handler
    ros::NodeHandle node_;
    ros::NodeHandle private_nh_;

    // Pub/Sub
    ros::Publisher entropy_publisher_;
    ros::Publisher map_publisher_;
    ros::Publisher map_meta_publisher_;
    ros::ServiceServer m_map_service_server;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;

    // TF Related.
    tf::TransformListener tf_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    tf::TransformBroadcaster* tfB_;
    tf::Stamped<tf::Pose> centered_laser_pose_;
    tf::Transform map_to_odom_;

    // ROS Transform Related.
    std::string m_base_frame, m_map_frame, m_odom_frame;  

    // GMapping Object
    GMapping::GridSlamProcessor* m_gsp;
    GMapping::RangeSensor* m_gsp_laser;
    GMapping::GridSlamProcessor::Params m_params;

    std::vector<double> m_laser_angles;
    nav_msgs::GetMap::Response map_;
    ros::Duration map_update_interval_;
    // Synch variables.
    boost::mutex m_map_to_odom_mutex, m_map_mutex;
    boost::thread* transform_thread_;

    bool m_got_first_scan, m_got_first_map, do_reverse_range_;
    unsigned int m_gsp_beam_cnt;
    unsigned long int seed_;    
    int m_laser_callback_called_cnt, m_laser_process_cycle;
    double m_occ_thresh, m_transform_publish_period, m_tf_delay;
    ros::Duration m_map_update_interval;
    double m_xmin, m_ymin, m_xmax, m_ymax; 
};
