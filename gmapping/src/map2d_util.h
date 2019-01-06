
#ifndef _MAP2D_UTIL_H_
#define _MAP2D_UTIL_H_

// ROS Header
#include "nav_msgs/GetMap.h"

// GMapping Header
#include "gmapping/scanmatcher/smmap.h"

namespace slam_gmapping {
namespace util {

void convertSmMap2RosMap(
  const GMapping::ScanMatcherMap& smmap, double occ_thresh, nav_msgs::GetMap::Response& ros_map);

};
};

#endif // _MAP2D_UTIL_H_