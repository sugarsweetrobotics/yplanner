#pragma once

#include <vector>
#include "yplanner/dimension.h"

#include "yplanner/map_2d.h"


#include "yplanner/costmap_2d.h"

namespace ssr {
  namespace yplanner {

    struct Waypoint2D {
      Time   timelimit;
      Pose2D pose;
      bool   allowHeading;
      bool   temporalStop;
      Point2D pathThroughTolerance;
      Waypoint2D() : timelimit(), pose(), allowHeading(false), temporalStop(false), pathThroughTolerance(-1, -1) {}
    };

    struct Path2D {
      std::vector<Waypoint2D> waypoints;
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
      };
      PLANNING_STATUS status;
      std::shared_ptr<const CostMap2DBase> costMap;
      std::vector<Path2D> paths;
      Plan(): status(PLAN_NG_UNKNOWN) {}
      Plan(PLANNING_STATUS stat): status(stat) {}
      Plan(const std::shared_ptr<const CostMap2DBase>& cmap, const std::vector<Path2D>& ps): status(PLAN_OK), costMap(cmap), paths(ps) {} 
    };

    struct RobotGeometry {
      float robotRadius;
    };

    struct PlannerConfig {
      Pose2D startPose;
      Pose2D goalPose;
      RobotGeometry robotGeometry;
    };

    Plan planPath(const OccupancyGridMap2D_ptr& map, const PlannerConfig& config);
    
    bool saveMapAndPathAsASCII(const OccupancyGridMap2D_ptr& map, const Path2D& path, const std::string& fileName);
        

  }


}
