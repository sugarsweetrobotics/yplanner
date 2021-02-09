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
      int8_t dmy;
      /**
       *
       */
      OccupancyGridMap2D(const MapConfig2D& c, const std::vector<int8_t>& cs): config(c), cells(cs) {}

      /**
       *
       */
      OccupancyGridMap2D(MapConfig2D&& c, std::vector<int8_t>&& cs): config(c), cells(cs) {}   

      int8_t& cell(const GridPoint2D& point);
      int8_t& cell(const int64_t x, const int64_t y);
      int8_t cell(const GridPoint2D& point) const;
      int8_t cell(const int64_t x, const int64_t y) const;
    };

    using OccupancyGridMap2D_ptr = std::shared_ptr<OccupancyGridMap2D>;

    /**
     * Create Map Object 
     */
    OccupancyGridMap2D_ptr createMap(const Pose2D& topLeft, const GridSize2D& mapGridSize, const Size2D& sizeOfGrid);

    /**
     * Convert Grid Pose ([pixel, rad]) in a map to World Pose [m, rad]
     * This function returns the pose in world coord of the center of the grid defined by gridPose (x, y, theta)
     */
    Point2D gridToWorld(const MapConfig2D& mapConfig, const GridPoint2D& gridPose);

    GridPoint2D worldToGrid(const MapConfig2D& mapConfig, const Point2D& point);

    inline int mapIndex(const MapConfig2D& config, const int x, const int y) {
        return x + y * config.cellsSize.width;
    }

    void forEachCells(const OccupancyGridMap2D_ptr& map, const std::function<void(int8_t& cell, const int index)>& f);

    bool isValidPoint(const MapConfig2D& mapConfig, const GridPoint2D& point);

    bool isFree(const OccupancyGridMap2D_ptr& map, const GridPoint2D& point);

    bool isOccupied(const OccupancyGridMap2D_ptr& map, const GridPoint2D& point);

    bool saveMapAsASCII(const OccupancyGridMap2D_ptr& map, const std::string& fileName);
    


    inline int8_t&  OccupancyGridMap2D::cell(const GridPoint2D& point) {
      if (!isValidPoint(config, point)) {
        return dmy;
      } 
      return cells[point.x + point.y * config.cellsSize.width];
    }

    inline int8_t& OccupancyGridMap2D::cell(const int64_t x, const int64_t y) {
      if (x < 0 || x >= config.cellsSize.width || y < 0 || y >= config.cellsSize.height) {
        return dmy;
      }
      return cells[x + y * config.cellsSize.width];
    }

    inline int8_t OccupancyGridMap2D::cell(const GridPoint2D& point) const {
      if (!isValidPoint(config, point)) {
        return dmy;
      } 
      return cells[point.x + point.y * config.cellsSize.width];
    }

    inline int8_t OccupancyGridMap2D::cell(const int64_t x, const int64_t y) const {
      if (x < 0 || x >= config.cellsSize.width || y < 0 || y >= config.cellsSize.height) {
        return dmy;
      }
      return cells[x + y * config.cellsSize.width];
    }



        OccupancyGridMap2D_ptr dilate(const OccupancyGridMap2D_ptr& map);

        OccupancyGridMap2D_ptr dilate(const OccupancyGridMap2D_ptr& map, const int iteration);

  }
}
