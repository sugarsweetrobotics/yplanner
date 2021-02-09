
#include <climits>
#include <cfloat>
#include <queue>
#include <optional>
#include <fstream>
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

        float AStar_costCalculator(const AStarCost& cost) {
            return cost.score();
        }

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
        //const auto gridHeuristicFunc = [](auto d) {return (fabs(d.v[0]) > fabs(d.v[1])) ? fabs(d.v[0]) : fabs(d.v[1]);}; //Chebyshev Distance
        const auto gridHeuristicFunc = [](auto d) {return fabs(d.v[0]) + fabs(d.v[1]); }; // Manhattan Distance
        bool expander9Dir(const AStarCostMap_ptr& costMap, const OccupancyGridMap2D_ptr& map, const GridPoint2D& pickedPoint, const GridPoint2D& goalPoint, const HeuristicFuncType& heuristicFunc) {
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

        std::shared_ptr<AStarCostMap> AStar_costMap(const OccupancyGridMap2D_ptr& map) {
            return std::make_shared<AStarCostMap>(map);
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

        Path2D AStar_gridPath(const std::shared_ptr<const AStarCostMap>& cmap, const GridPoint2D& startPoint, const GridPoint2D& goalPoint) {
            Path2D path;
            auto pickedPoint = goalPoint;
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

        OccupancyGridMap2D_ptr dilate(const OccupancyGridMap2D_ptr& map) {
            auto expMap = createMap(map->config.poseOfTopLeft, map->config.cellsSize, map->config.gridSize);
            for(int h = 0;h < map->config.cellsSize.height;h++) {
                for(int w = 0;w < map->config.cellsSize.width;w++) {
                    if (map->cell(w, h) == OccupancyGridMap2D::OCCUPIED_STATE) {
                        expMap->cell(w-1, h-1) = OccupancyGridMap2D::OCCUPIED_STATE;
                        expMap->cell(w-1, h) = OccupancyGridMap2D::OCCUPIED_STATE;
                        expMap->cell(w-1, h+1) = OccupancyGridMap2D::OCCUPIED_STATE;

                        expMap->cell(w, h-1) = OccupancyGridMap2D::OCCUPIED_STATE;
                        expMap->cell(w, h) = OccupancyGridMap2D::OCCUPIED_STATE;
                        expMap->cell(w, h+1) = OccupancyGridMap2D::OCCUPIED_STATE;

                        expMap->cell(w+1, h-1) = OccupancyGridMap2D::OCCUPIED_STATE;
                        expMap->cell(w+1, h) = OccupancyGridMap2D::OCCUPIED_STATE;
                        expMap->cell(w+1, h+1) = OccupancyGridMap2D::OCCUPIED_STATE;
                    } else {
                        if (expMap->cell(w, h) != OccupancyGridMap2D::OCCUPIED_STATE) {
                            expMap->cell(w, h) = map->cell(w, h);
                        }
                    }
                }
            }
            return expMap;
        }


        OccupancyGridMap2D_ptr dilate(const OccupancyGridMap2D_ptr& map, const int iteration) {
            if (iteration < 1) { return map; }

            OccupancyGridMap2D_ptr eMap = dilate(map);
            for(int i = 0;i < iteration-1;i++) {
                eMap = dilate(eMap);
            }
            return eMap;
        }

        OccupancyGridMap2D_ptr mapExpander(const OccupancyGridMap2D_ptr& map, const RobotGeometry& robotGeometry) {
            auto radiusByGrid = robotGeometry.robotRadius / ((map->config.gridSize.width + map->config.gridSize.height)/2);
            return dilate(map, radiusByGrid);
        }

        Plan planPath(const OccupancyGridMap2D_ptr& map, const PlannerConfig& config) {
            const auto startPoint = worldToGrid(map->config, config.startPose.position) ;
            if (!isValidPoint(map->config, startPoint)) {
                return {Plan::PLAN_NG_START_IS_OUTOFMAP}; // スタート位置がマップ外
            }
            const auto goalPoint = worldToGrid(map->config, config.goalPose.position) ;
            if (!isValidPoint(map->config, goalPoint)) {
                return {Plan::PLAN_NG_GOAL_IS_OUTOFMAP}; // ゴール位置がマップ外
            }
            if (goalPoint == startPoint) {
                return {Plan::PLAN_NG_START_IS_GOAL};
            }

            auto expandedMap = mapExpander(map, config.robotGeometry);

            if (expandedMap->cell(startPoint) == OccupancyGridMap2D::OCCUPIED_STATE) {
                return {Plan::PLAN_NG_START_IS_OCCUPIEDSTATE}; // スタート位置が障害物上
            }
            if (expandedMap->cell(goalPoint) == OccupancyGridMap2D::OCCUPIED_STATE) {
                return {Plan::PLAN_NG_GOAL_IS_OCCUPIEDSTATE}; // ゴール位置が障害物上
            }
            auto costMap = AStar_costMap(map);
            auto cmap = AStar_calculateCosts(costMap, expandedMap, startPoint, goalPoint, gridHeuristicFunc, expander9Dir);
            if (cmap == nullptr) {
                return {Plan::PLAN_NG_NOROUTE}; // ゴールまでの経路がない．
            }
            
            return {cmap, {AStar_gridPath(cmap, startPoint, goalPoint)}};
        }

        /// Save as PCM
        bool saveMapAndPathAsASCII(const OccupancyGridMap2D_ptr& map, const Path2D& path, const std::string& fileName) {
            std::ofstream fout(fileName);
            fout << "P3" << std::endl;
            fout << "#" << std::endl;
            fout << map->config.cellsSize.width << " " << map->config.cellsSize.height << std::endl;
            fout << 255 << std::endl;
            for(int i = 0;i < map->config.cellsSize.height;i++) {
                for(int j = 0;j < map->config.cellsSize.width;j++) {
                    int index = mapIndex(map->config, j, i);
                    bool isOnWaypoint = false;
                    for (auto wp : path.waypoints) { //int p = 0;p < path.waypoints.size();p++) {
                        GridPoint2D gwp = worldToGrid(map->config, wp.pose.position);
                        if (gwp.x == j && gwp.y == i) {
                            isOnWaypoint = true;
                        }
                    }
                    if (isOnWaypoint) {
                        fout << "255 0 0 ";
                    } else {
                        int32_t info = 255; // FREE
                        if (map->cells[index] == map->UNKNOWN_STATE) {
                            info = 125;
                        } else if (map->cells[index] == map->OCCUPIED_STATE) {
                            info = 0;
                        }
                        fout << info << " " << info << " " << info << " ";
                    }
                }
                fout << std::endl;
            }
            return true;
        }


        
    }
}

/*

function reconstruct_path(cameFrom, current)
    total_path := {current}
    while current in cameFrom.Keys:
        current := cameFrom[current]
        total_path.prepend(current)
    return total_path

// A* finds a path from start to goal.
// h is the heuristic function. h(n) estimates the cost to reach goal from node n.
function A_Star(start, goal, h)
    // The set of discovered nodes that may need to be (re-)expanded.
    // Initially, only the start node is known.
    // This is usually implemented as a min-heap or priority queue rather than a hash-set.
    openSet := {start}

    // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
    // to n currently known.
    cameFrom := an empty map

    // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore := map with default value of Infinity
    gScore[start] := 0

    // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    // how short a path from start to finish can be if it goes through n.
    fScore := map with default value of Infinity
    fScore[start] := h(start)

    while openSet is not empty
        // This operation can occur in O(1) time if openSet is a min-heap or a priority queue
        current := the node in openSet having the lowest fScore[] value
        if current = goal
            return reconstruct_path(cameFrom, current)

        openSet.Remove(current)
        for each neighbor of current
            // d(current,neighbor) is the weight of the edge from current to neighbor
            // tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore := gScore[current] + d(current, neighbor)
            if tentative_gScore < gScore[neighbor]
                // This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] := current
                gScore[neighbor] := tentative_gScore
                fScore[neighbor] := gScore[neighbor] + h(neighbor)
                if neighbor not in openSet
                    openSet.add(neighbor)

    // Open set is empty but goal was never reached
    return failure

    */