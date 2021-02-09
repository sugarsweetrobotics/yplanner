
#include <climits>
#include <cfloat>
#include <queue>
#include <optional>
#include <fstream>
#include "yplanner/costmap_2d.h"
#include "yplanner/yplanner.h"
#include "astar.h"

namespace ssr {
    namespace yplanner {

        Plan simplifyPlanPath(const Plan& plan, const PlannerConfig& config) {
            if (plan.status != Plan::PLAN_OK) {
                return plan;
            }
            return {plan.status, plan.costMap, {simplifyPath(plan.paths[0], config.pathFilterConfig)}};
        }

        Path2D simplifyPath(const Path2D& path, const PathFilterConfig& config) {
            return path;
        }


        OccupancyGridMap2D_ptr mapExpander(const OccupancyGridMap2D_ptr& map, const RobotGeometry& robotGeometry) {
            auto radiusByGrid = robotGeometry.robotRadius / ((map->config.gridSize.width + map->config.gridSize.height)/2);
            return dilate(map, radiusByGrid);
        }

        Plan planPath(const OccupancyGridMap2D_ptr& map, const PlannerConfig& config) {
            // 世界座標系の位置をマップ上のグリッドに変換
            const auto startPoint = worldToGrid(map->config, config.startPose.position);
            const auto goalPoint = worldToGrid(map->config, config.goalPose.position);

            // スタートとゴールがマップ内にあるかどうかを確認
            if (!isValidPoint(map->config, startPoint)) return {Plan::PLAN_NG_START_IS_OUTOFMAP}; // スタート位置がマップ外
            if (!isValidPoint(map->config, goalPoint)) return {Plan::PLAN_NG_GOAL_IS_OUTOFMAP}; // ゴール位置がマップ外

            // スタートとゴールが重なっていればパスは作らない
            if (goalPoint == startPoint) return {Plan::PLAN_NG_START_IS_GOAL};

            // ロボットの半径分だけマップの障害物を大きくしておく
            // 以降はロボットを質点として扱える
            auto expandedMap = mapExpander(map, config.robotGeometry);

            // ロボット中心がOCCUPIEDであればすでに障害物に接触ないしはめり込んでいるのでエラー
            if (expandedMap->cell(startPoint) == OccupancyGridMap2D::OCCUPIED_STATE) return {Plan::PLAN_NG_START_IS_OCCUPIEDSTATE}; // スタート位置が障害物上
            if (expandedMap->cell(goalPoint) == OccupancyGridMap2D::OCCUPIED_STATE) return {Plan::PLAN_NG_GOAL_IS_OCCUPIEDSTATE}; // ゴール位置が障害物上

            if(config.method == PlannerConfig::PLANNING_ASTAR || config.method == PlannerConfig::PLANNING_ASTAR_8DIR) {
                // A*アルゴリズムでコストを計算
                auto cmap = AStar_calculateCosts(AStar_costMap(map), expandedMap, startPoint, goalPoint, config);
                if (cmap == nullptr) {
                    return {Plan::PLAN_NG_NOROUTE}; // ゴールまでの経路がない．
                }
                return {cmap, {AStar_gridPath(cmap, startPoint, goalPoint)}};
            }
            return {Plan::PLAN_NG_INVALID_METHOD};
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
