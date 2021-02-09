#pragma once

#include <vector>
#include "yplanner/dimension.h"

#include "yplanner/map_2d.h"


#include "yplanner/costmap_2d.h"

namespace ssr {
  namespace yplanner {
    struct PathFilterConfig {

    };

    /**
     * 経路計画をするためのパラメータ
     */
    struct PlannerConfig {
      enum PLANNING_METHOD {
        PLANNING_ASTAR=0, // グリッドで移動．上下左右の４方向のみ
        PLANNING_ASTAR_4DIR=0, // グリッドで移動．上下左右の４方向のみ
        PLANNING_ASTAR_8DIR=1, // グリッドで斜め移動を許可する．8方向に移動できる
      };
      PLANNING_METHOD method; // 経路計画アルゴリズムの選択

      PathFilterConfig pathFilterConfig;

      Pose2D startPose;
      Pose2D goalPose;
      RobotGeometry robotGeometry;
      bool allowHeading; // 生成されたPathにロボットの向きに追従すべきという情報を含めるか否か．
      Velocity2D meanRobotVelocity; // ロボットの平均移動速度．ここからタイムリミットを算出する
      double distanceTolerance;
      PlannerConfig(const Pose2D& start, const Pose2D& goal, const RobotGeometry& geom): method(PLANNING_ASTAR_8DIR), startPose(start), goalPose(goal), robotGeometry(geom), allowHeading(false), meanRobotVelocity(-1, -1, -1), distanceTolerance(-1) {}
    };


    struct Waypoint2D {
      Time   timelimit;
      Pose2D pose;
      bool   allowHeading;
      bool   temporalStop;
      double distanceTolerance;
      Waypoint2D() : timelimit(), pose(), allowHeading(false), temporalStop(false), distanceTolerance(-1) {}
      Waypoint2D(const double x, const double y): timelimit(), pose(x, y, 0), allowHeading(false), temporalStop(false), distanceTolerance(-1) {}
      Waypoint2D(const Point2D& point, const double tolerance): timelimit(), pose(point, 0), allowHeading(false), temporalStop(false), distanceTolerance(tolerance) {}
    };

    struct Path2D {
      std::vector<Waypoint2D> waypoints;
      Path2D() {}
      Path2D(const std::initializer_list<Waypoint2D>& list): waypoints(list) {}
    };

    struct Plan {
      enum PLANNING_STATUS {
        PLAN_NG_UNKNOWN,
        PLAN_OK = 1,
        PLAN_NG_START_IS_OUTOFMAP = 2,
        PLAN_NG_GOAL_IS_OUTOFMAP = 3,
        PLAN_NG_START_IS_OCCUPIEDSTATE = 4,
        PLAN_NG_GOAL_IS_OCCUPIEDSTATE = 5,
        PLAN_NG_START_IS_GOAL = 6,
        PLAN_NG_NOROUTE = 7,
        PLAN_NG_INVALID_METHOD = 8,
      };

      PLANNING_STATUS status;
      std::shared_ptr<const CostMap2DBase> costMap;
      std::vector<Path2D> paths;

      Plan(): status(PLAN_NG_UNKNOWN) {}
      Plan(const Plan& plan): status(plan.status), costMap(plan.costMap), paths(plan.paths) {}
      Plan(PLANNING_STATUS stat): status(stat) {}
      Plan(const std::shared_ptr<const CostMap2DBase>& cmap, const std::vector<Path2D>& ps): status(PLAN_OK), costMap(cmap), paths(ps) {} 
      Plan(const PLANNING_STATUS& status, const std::shared_ptr<const CostMap2DBase>& cmap, const std::vector<Path2D>& ps): status(status), costMap(cmap), paths(ps) {} 
    };


    Plan planPath(const OccupancyGridMap2D_ptr& map, const PlannerConfig& config);
    
    bool saveMapAndPathAsASCII(const OccupancyGridMap2D_ptr& map, const Path2D& path, const std::string& fileName);
        
    Path2D simplifyPath(const Path2D& path, const PathFilterConfig& config);
    Plan simplifyPlanPath(const Plan& plan, const PlannerConfig& config);
  }


}
