
#include "map2d_util.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace slam_gmapping {
namespace util {

void convertSmMap2RosMap(
  const GMapping::ScanMatcherMap& smmap, double occ_thresh, nav_msgs::GetMap::Response& ros_map) {
    
    int map_size_x = smmap.getMapSizeX();
    int map_size_y = smmap.getMapSizeY();

    if (  ros_map.map.info.width != static_cast<uint32_t>(map_size_x)
      ||  ros_map.map.info.height != static_cast<uint32_t>(map_size_y)) {
      
      GMapping::Point wmin = smmap.map2world(GMapping::IntPoint(0, 0));
      GMapping::Point wmax = smmap.map2world(GMapping::IntPoint(map_size_x, map_size_y));

      ros_map.map.info.width = map_size_x;
      ros_map.map.info.height = map_size_y;
      ros_map.map.info.origin.position.x = wmin.x;
      ros_map.map.info.origin.position.y = wmin.y;
      ros_map.map.data.resize(ros_map.map.info.width * ros_map.map.info.height);

    }

    for (int x = 0; x < ros_map.map.info.width; x++) {
      for (int y = 0; y < ros_map.map.info.height; y++) {
        
        GMapping::IntPoint p(x, y);
        double occ = smmap.cell(p);
        assert(occ <= 1.0);

        if (occ < 0) {
          ros_map.map.data[MAP_IDX(ros_map.map.info.width, x, y)] = -1;
        } else if (occ > occ_thresh) {
          ros_map.map.data[MAP_IDX(ros_map.map.info.width, x, y)] = 100;
        } else {
          ros_map.map.data[MAP_IDX(ros_map.map.info.width, x, y)] = 0;
        }
      }
    }
  }

};
};