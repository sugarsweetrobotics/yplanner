#include "yplanner/map_2d.h"
#include <fstream>

namespace ssr {

  namespace yplanner {

    /**
     * Create Map Object 
     */
    OccupancyGridMap2D_ptr createMap(const Pose2D& topLeft, const GridSize2D& mapGridSize, const Size2D& sizeOfGrid) {
      const uint32_t w = mapGridSize.width;// * sizeOfGrid.width;
      const uint32_t h = mapGridSize.height;// * sizeOfGrid.height;
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
    Point2D gridToWorld(const MapConfig2D& mapConfig, const GridPoint2D& gridPose) {
      const double x = (gridPose.x+0.5) * mapConfig.gridSize.width;
      const double y = -(gridPose.y+0.5) * mapConfig.gridSize.height;
      const double sin_a = sin(mapConfig.poseOfTopLeft.a);
      const double cos_a = cos(mapConfig.poseOfTopLeft.a);
      return {mapConfig.poseOfTopLeft.position.x + x * cos_a - y * sin_a,
	      mapConfig.poseOfTopLeft.position.y + x * sin_a + y * cos_a};
    }

    GridPoint2D worldToGrid(const MapConfig2D& mapConfig, const Point2D& point) {
        return {static_cast<uint32_t>(floor((point.x - mapConfig.poseOfTopLeft.position.x ) / mapConfig.gridSize.width)),
         static_cast<uint32_t>(floor((- point.y + mapConfig.poseOfTopLeft.position.y) / mapConfig.gridSize.height))};
    }

    bool isValidPoint(const MapConfig2D& mapConfig, const GridPoint2D& point) {
        if (point.x < 0 || point.x >= mapConfig.cellsSize.width ||
            point.y < 0 || point.y >= mapConfig.cellsSize.height) {
            return false;
        }
        return true;
    }

    bool isFree(const OccupancyGridMap2D_ptr& map, const GridPoint2D& point) {
        if (!isValidPoint(map->config, point)) {
            return false;
        }
        return map->cells[point.x + point.y * map->config.cellsSize.height] == map->FREE_STATE;
    }
        
    bool isOccupied(const OccupancyGridMap2D_ptr& map, const GridPoint2D& point) {
        if (!isValidPoint(map->config, point)) {
            return false;
        }
        return map->cells[point.x + point.y * map->config.cellsSize.height] == map->OCCUPIED_STATE;
    }

    bool isUnknown(const OccupancyGridMap2D_ptr& map, const GridPoint2D& point) {
        if (!isValidPoint(map->config, point)) {
            return false;
        }
        return map->cells[point.x + point.y * map->config.cellsSize.height] == map->UNKNOWN_STATE;
    }

    void forEachCells(OccupancyGridMap2D_ptr& map, const std::function<void(int8_t& cell, const int index)>& f) {
        for(int i = 0;i < map->config.cellsSize.height;i++) {
            for(int j = 0;j < map->config.cellsSize.width;j++) {
                int index = mapIndex(map->config, j, i);
                f(map->cells[index], index);
            }
        }
    }

    bool saveMapAsASCII(const OccupancyGridMap2D_ptr& map, const std::string& fileName) {
        std::ofstream fout(fileName);
        fout << "P2" << std::endl;
        fout << "#" << std::endl;
        fout << map->config.cellsSize.width << " " << map->config.cellsSize.height << std::endl;
        fout << 255 << std::endl;
        for(int i = 0;i < map->config.cellsSize.height;i++) {
            for(int j = 0;j < map->config.cellsSize.width;j++) {
                int index = mapIndex(map->config, j, i);
                int32_t info = 255; // FREE
                if (map->cells[index] == map->UNKNOWN_STATE) {
                    info = 125;
                } else if (map->cells[index] == map->OCCUPIED_STATE) {
                    info = 0;
                }
                fout << info << " ";
            }
            fout << std::endl;
        }
        return true;
    }


  }
}