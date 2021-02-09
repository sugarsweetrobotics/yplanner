#pragma once

#include <memory>
#include <vector>
#include "yplanner/map_2d.h"
namespace ssr {
  namespace yplanner {

    struct Cost {
      float cost;
    };
    
    class CostMap2DBase {
      virtual float getCost(const int x, const int y) const = 0;
    };
    template<typename T>
    struct CostMap2D : public CostMap2DBase {
    public:
      const OccupancyGridMap2D_ptr originalMap;
    private:
      const std::function<float(const T&)> costCalculator_;
      std::vector<T> cells;

    public:
      CostMap2D(const OccupancyGridMap2D_ptr& map, const std::function<float(const T&)>& costCalculator): originalMap(map), costCalculator_(costCalculator), cells(map->config.cellsSize.width * map->config.cellsSize.height) {}

      CostMap2D(const OccupancyGridMap2D_ptr& map, const std::vector<T>& cells, const std::function<float(const T&)>& costCalculator): originalMap(map), costCalculator_(costCalculator), cells(cells) {}

      virtual ~CostMap2D() {}

      virtual float getCost(const int x, const int y) const override {
        if ( (x < 0 || x >= originalMap->config.cellsSize.width) ||
             (y < 0 || y >= originalMap->config.cellsSize.height) ) {
          return -1.;
        }
        return costCalculator_(cells[x + y * originalMap->config.cellsSize.width]);
      }

      T& cell(const GridPoint2D& point) {
        return cells[point.x + point.y * originalMap->config.cellsSize.width];
      }

      T cell(const GridPoint2D& point) const {
        return cells[point.x + point.y * originalMap->config.cellsSize.width];
      }
    };

    //using CostMap2D_ptr = std::shared_ptr<CostMap2D>;

    //CostMap2D_ptr costMap(const OccupancyGridMap2D_ptr& map);
    
  }
}
