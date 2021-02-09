#pragma once



#include <climits>
#include <cfloat>
#include <queue>
#include <optional>
#include <fstream>
#include "yplanner/costmap_2d.h"
#include "yplanner/yplanner.h"

namespace ssr {
    namespace yplanner {

        std::shared_ptr<CostMap2DBase> AStar_costMap(const OccupancyGridMap2D_ptr& map);

        std::shared_ptr<const CostMap2DBase> AStar_calculateCosts(const std::shared_ptr<CostMap2DBase>& cmap, const OccupancyGridMap2D_ptr& map, const GridPoint2D& startPoint, const GridPoint2D& goalPoint, const PlannerConfig& config);

        Path2D AStar_gridPath(const std::shared_ptr<const CostMap2DBase>& cmap, const GridPoint2D& startPoint, const GridPoint2D& goalPoint);
        
    }
}
