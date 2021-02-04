#pragma once

#include <cmath>
#include <vector>
#include <memory>
#include "yplanner/dimension.h"

namespace ssr {

  namespace yplanner {

    /**
     *
     */
    struct MapConfig2D {
      Pose2D poseOfTopLeft;
      Size2D gridSize;
      GridSize2D cellsSize;
    };

    /**
     *
     */
    struct OccupancyGridMap2D {
    public:
      static const int8_t UNKNOWN_STATE = 0;
      static const int8_t OCCUPIED_STATE = 1;
      static const int8_t FREE_STATE = -1;
    public:
      MapConfig2D config;
      std::vector<int8_t> cells;

      /**
       *
       */
      OccupancyGridMap2D(const MapConfig2D& c, const std::vector<int8_t>& cs): config(c), cells(cs) {}

      /**
       *
       */
      OccupancyGridMap2D(MapConfig2D&& c, std::vector<int8_t>&& cs): config(c), cells(cs) {}      
    };

    using OccupancyGridMap2D_ptr = std::shared_ptr<OccupancyGridMap2D>;

    /**
     * Create Map Object 
     */
    OccupancyGridMap2D_ptr createMap(const Pose2D& topLeft, const Size2D& mapSize, const Size2D& sizeOfGrid) {
      const uint32_t w = mapSize.width / sizeOfGrid.width;
      const uint32_t h = mapSize.height / sizeOfGrid.height;
      auto m = std::make_shared<OccupancyGridMap2D>(MapConfig2D{topLeft, sizeOfGrid, {w, h}}, std::vector<int8_t>(static_cast<std::vector<int8_t>::size_type>(w*h)));
      for(auto i = 0;i < w*h;i++) {
	m->cells[i] = OccupancyGridMap2D::UNKNOWN_STATE;
      }
      return m;
    }

    /**
     * Convert Grid Pose ([pixel, rad]) in a map to World Pose [m, rad]
     * This function returns the pose in world coord of the center of the grid defined by gridPose (x, y, theta)
     */
    inline Point2D gridPoseToWorldPose(const MapConfig2D& mapConfig, const GridPoint2D& gridPose) {
      const double x = (gridPose.x+0.5) * mapConfig.gridSize.width;
      const double y = -(gridPose.y+0.5) * mapConfig.gridSize.height;
      const double sin_a = sin(mapConfig.poseOfTopLeft.a);
      const double cos_a = cos(mapConfig.poseOfTopLeft.a);
      return {mapConfig.poseOfTopLeft.position.x + x * cos_a - y * sin_a,
	mapConfig.poseOfTopLeft.position.y + x * sin_a + y * cos_a};
    }
    
  }
}
