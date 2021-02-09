
#include <climits>
#include <cfloat>
#include <queue>
#include <optional>
#include <fstream>
#include "astar.h"

#include "yplanner/costmap_2d.h"
#include "yplanner/yplanner.h"

namespace ssr {
    namespace yplanner {

        struct AStarCost {
            enum OPEN_CLOSE_STATE {
                NONE_STATE,
                OPEN_STATE,
                CLOSE_STATE
            };

            float c_;
            float h_;
            GridPoint2D parentPoint_;
            OPEN_CLOSE_STATE state_;

            AStarCost(): c_(0), h_(0), state_(NONE_STATE) {} 

            float score() const { return c_ + h_; }
        };

        float AStar_costCalculator(const AStarCost& cost);

        class AStarCostMap : public CostMap2D<AStarCost> {
        public:
            AStarCostMap(const OccupancyGridMap2D_ptr& map): CostMap2D<AStarCost>(map, AStar_costCalculator), 
              openedPointList{[this](const GridPoint2D& x0, const GridPoint2D& x1) {
                return this->cell(x0).score() > this->cell(x1).score();
            }} {} 

            std::priority_queue<GridPoint2D, std::vector<GridPoint2D>, std::function<bool(const GridPoint2D&, const GridPoint2D&)>> openedPointList;
        };

        using AStarCostMap_ptr = std::shared_ptr<AStarCostMap>;
        using HeuristicFuncType = std::function<double(const GridVector2D&)>;
        using ExpanderFuncType = std::function<bool(const AStarCostMap_ptr&, const OccupancyGridMap2D_ptr&, const GridPoint2D&, const GridPoint2D&, const HeuristicFuncType&)>;
        

        std::shared_ptr<const AStarCostMap> AStar_calculateCosts(const std::shared_ptr<AStarCostMap>& cmap, const OccupancyGridMap2D_ptr& map, const GridPoint2D& startPoint, const GridPoint2D& goalPoint, const HeuristicFuncType& heuristicFunc, const ExpanderFuncType& expander);

        float AStar_costCalculator(const AStarCost& cost) {
            return cost.score();
        }
        
        GridPoint2D openPoint(const AStarCostMap_ptr& costMap, 
                                                const GridPoint2D& openingPoint,
                                                const GridPoint2D& pickedPoint, 
                                                const GridPoint2D& goalPoint,
                                                const HeuristicFuncType& heuristicFunction
                                                ) {
            if (!isValidPoint(costMap->originalMap->config, openingPoint)) {
                return openingPoint;
            }
            if (isOccupied(costMap->originalMap, openingPoint)) { 
                return openingPoint;
            }
            if (costMap->cell(openingPoint).state_ != AStarCost::NONE_STATE) {
                return openingPoint;
            }

            costMap->cell(openingPoint).c_ = costMap->cell(pickedPoint).c_ + heuristicFunction(openingPoint-pickedPoint);
            costMap->cell(openingPoint).h_ = heuristicFunction(goalPoint-openingPoint);
            costMap->cell(openingPoint).parentPoint_ = pickedPoint;
            costMap->cell(openingPoint).state_ = AStarCost::OPEN_STATE;
            costMap->openedPointList.push(openingPoint);
            return openingPoint;
        }

        /// 斜め移動が可能な時
        const auto chebyshevDistance = [](auto d) {return (fabs(d.v[0]) > fabs(d.v[1])) ? fabs(d.v[0]) : fabs(d.v[1]);}; //Chebyshev Distance
        const auto manhattanDistance = [](auto d) {return fabs(d.v[0]) + fabs(d.v[1]); }; // Manhattan Distance


        bool expander4Dir(const AStarCostMap_ptr& costMap, const OccupancyGridMap2D_ptr& map, const GridPoint2D& pickedPoint, const GridPoint2D& goalPoint, const HeuristicFuncType& heuristicFunc) {
            costMap->cell(pickedPoint).state_ = AStarCost::CLOSE_STATE;
            const GridPoint2D N{pickedPoint.x, pickedPoint.y-1};
            const GridPoint2D S{pickedPoint.x, pickedPoint.y+1};
            const GridPoint2D E{pickedPoint.x+1, pickedPoint.y};
            const GridPoint2D W{pickedPoint.x-1, pickedPoint.y};
            if (map->cell(N) != OccupancyGridMap2D::OCCUPIED_STATE) {
                if(openPoint(costMap, N, pickedPoint, goalPoint, heuristicFunc) == goalPoint) return true; // If goal arrived
            }
            if (map->cell(S) != OccupancyGridMap2D::OCCUPIED_STATE) {
                if(openPoint(costMap, S, pickedPoint, goalPoint, heuristicFunc) == goalPoint) return true; // If goal arrived
            }
            if (map->cell(E) != OccupancyGridMap2D::OCCUPIED_STATE) {
                if(openPoint(costMap, E, pickedPoint, goalPoint, heuristicFunc) == goalPoint) return true; // If goal arrived
            }
            if (map->cell(W) != OccupancyGridMap2D::OCCUPIED_STATE) {
                if(openPoint(costMap, W, pickedPoint, goalPoint, heuristicFunc) == goalPoint) return true; // If goal arrived
            }
            return false;
        }


        bool expander8Dir(const AStarCostMap_ptr& costMap, const OccupancyGridMap2D_ptr& map, const GridPoint2D& pickedPoint, const GridPoint2D& goalPoint, const HeuristicFuncType& heuristicFunc) {
            costMap->cell(pickedPoint).state_ = AStarCost::CLOSE_STATE;
            for(int i = -1;i < 2;i++) {
                for(int j = -1;j < 2;j++) {
                    if (i == 0 && j == 0) continue;
                    if((int64_t)pickedPoint.x+i < 0 || (int64_t)pickedPoint.y+j < 0) continue;
                    const GridPoint2D expandededPoint{pickedPoint.x+i, pickedPoint.y+j};

                    if (map->cell(expandededPoint) == OccupancyGridMap2D::OCCUPIED_STATE) continue;
                    if(openPoint(costMap, expandededPoint, pickedPoint, goalPoint, heuristicFunc) == goalPoint) {
                        return true; // If goal arrived
                    }
                }
            }
            return false;
        }


        std::optional<GridPoint2D> pickPoint(const AStarCostMap_ptr& costMap) {
            while(true) {
                auto openingPoint = costMap->openedPointList.top();
                costMap->openedPointList.pop();
                if (costMap->cell(openingPoint).state_ == AStarCost::OPEN_STATE) { return openingPoint; }
            }
            return std::nullopt;
        }

        std::shared_ptr<CostMap2DBase> AStar_costMap(const OccupancyGridMap2D_ptr& map) {
            return std::make_shared<AStarCostMap>(map);
        }


        std::shared_ptr<const CostMap2DBase> AStar_calculateCosts(const std::shared_ptr<CostMap2DBase>& cmap, const OccupancyGridMap2D_ptr& map, const GridPoint2D& startPoint, const GridPoint2D& goalPoint, const PlannerConfig& config) {
            if (config.method == PlannerConfig::PLANNING_ASTAR_8DIR) {
                return AStar_calculateCosts(std::dynamic_pointer_cast<AStarCostMap>(cmap), map, startPoint, goalPoint, chebyshevDistance, expander8Dir);
            } else if (config.method == PlannerConfig::PLANNING_ASTAR) {
                return AStar_calculateCosts(std::dynamic_pointer_cast<AStarCostMap>(cmap), map, startPoint, goalPoint, chebyshevDistance, expander4Dir);
            }
            return nullptr;
        }

        std::shared_ptr<const AStarCostMap> AStar_calculateCosts(const std::shared_ptr<AStarCostMap>& cmap, const OccupancyGridMap2D_ptr& map, const GridPoint2D& startPoint, const GridPoint2D& goalPoint, const HeuristicFuncType& heuristicFunc, const ExpanderFuncType& expander) {
            // スタート位置の指定
            openPoint(cmap, startPoint, startPoint, goalPoint, heuristicFunc);
            GridPoint2D pickedPoint = startPoint;
            while(true) {
                if (expander(cmap, map, pickedPoint, goalPoint, heuristicFunc)) { // arrive goal
                    break;
                } 
                auto pickedPointOpt = pickPoint(cmap);
                if (!pickedPointOpt) {
                    //すべてのポイントがCLOSEになったのにゴールにつかない
                    return nullptr;
                }
                pickedPoint = pickedPointOpt.value();
            }

            return cmap;
        }

        Path2D AStar_gridPath(const std::shared_ptr<const CostMap2DBase>& costMap, const GridPoint2D& startPoint, const GridPoint2D& goalPoint) {
            Path2D path;
            auto pickedPoint = goalPoint;
            auto cmap = std::dynamic_pointer_cast<const AStarCostMap>(costMap);
            while (pickedPoint != startPoint) {
                Waypoint2D wp;
                wp.pose = {gridToWorld(cmap->originalMap->config, pickedPoint), 0};
                path.waypoints.insert(path.waypoints.begin(), wp);
                pickedPoint = cmap->cell(pickedPoint).parentPoint_;
            }
            Waypoint2D wp;
            wp.pose = {gridToWorld(cmap->originalMap->config, startPoint), 0};
            path.waypoints.insert(path.waypoints.begin(), wp);

            return path;
        }
        
    }
}
