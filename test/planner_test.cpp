
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>
#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.hpp>


#include "yplanner/map_2d.h"
#include "yplanner/yplanner.h"


using namespace ssr;
using namespace Catch::literals;


SCENARIO( "Map test", "[constmap]" ) {


  GIVEN("MapConfiguration by code with 2x2 map") {
    auto map = yplanner::createMap({{0., 0.2}, 0.0}, {2, 2}, {0.1, 0.1});
    map->cells[0] = map->FREE_STATE;
    map->cells[1] = map->OCCUPIED_STATE;
    map->cells[2] = map->FREE_STATE;
    map->cells[3] = map->FREE_STATE;
    saveMapAsASCII(map, "temp_map2x2.pgm");

    THEN("Do plan with robot zero radius") {
      auto plan = ssr::yplanner::planPath(map, {
          {0.05, 0.15, 0}, {0.15, 0.05, 0}, {0.0}
      });
      REQUIRE(plan.status == ssr::yplanner::Plan::PLAN_OK);
      REQUIRE(plan.paths.size() > 0);

      saveMapAndPathAsASCII(map, plan.paths[0], "result2x2.pcm");
        
    }
  }

  GIVEN("MapConfiguration by code with 3x3 map") {
    auto map = yplanner::createMap({{0., 0.3}, 0.0}, {3, 3}, {0.1, 0.1});
    map->cells[0] = map->FREE_STATE;
    map->cells[1] = map->FREE_STATE;
    map->cells[2] = map->FREE_STATE;

    map->cells[3] = map->FREE_STATE;
    map->cells[4] = map->OCCUPIED_STATE;
    map->cells[5] = map->FREE_STATE;

    map->cells[6] = map->FREE_STATE;
    map->cells[7] = map->FREE_STATE;
    map->cells[8] = map->FREE_STATE;
    saveMapAsASCII(map, "temp_map3x3.pgm");

    THEN("Do Plan with robot zero radius") {
      auto plan = ssr::yplanner::planPath(map, {
          {0.05, 0.05, 0}, {0.25, 0.25, 0}, {0.0}
      });
      REQUIRE(plan.status == ssr::yplanner::Plan::PLAN_OK);
      REQUIRE(plan.paths.size() > 0);
      saveMapAndPathAsASCII(map, plan.paths[0], "result3x3.pcm");
    }

    THEN("DO Plan but map is too small") {
      auto plan = ssr::yplanner::planPath(map, {
          {0.05, 0.05, 0}, {0.25, 0.25, 0}, {0.1}
      });
      REQUIRE(plan.status != ssr::yplanner::Plan::PLAN_OK);
      REQUIRE(plan.paths.size() == 0);

    }
  }


  GIVEN("MapConfiguration by code with 7x7 map") {
    auto map = yplanner::createMap({{0., 0.7}, 0.0}, {7, 7}, {0.1, 0.1});
    const int8_t F = ssr::yplanner::OccupancyGridMap2D::FREE_STATE;
    map->cells = {
      1, 1, 1, 1, 1, 1, 1,
      1, F, F, F, F, F, 1, 
      1, F, F, F, F, F, 1, 
      1, F, F, F, F, F, 1, 
      1, F, F, F, F, F, 1, 
      1, F, F, F, F, F, 1, 
      1, 1, 1, 1, 1, 1, 1
    };
    saveMapAsASCII(map, "temp_map7x7.pgm");

    THEN("Do Plan with robot zero radius") {
      auto plan = ssr::yplanner::planPath(map, {
          {0.3, 0.3, 0}, {0.4, 0.4, 0}, {0.0}
      });
      REQUIRE(plan.status == ssr::yplanner::Plan::PLAN_OK);
      REQUIRE(plan.paths.size() > 0);
      saveMapAndPathAsASCII(map, plan.paths[0], "result7x7.pcm");
    }

    THEN("Do Plan with robot 0.1 radius") {
      auto plan = ssr::yplanner::planPath(map, {
          {0.3, 0.3, 0}, {0.4, 0.4, 0}, {0.1}
      });
      REQUIRE(plan.status == ssr::yplanner::Plan::PLAN_OK);
      REQUIRE(plan.paths.size() > 0);
      saveMapAndPathAsASCII(map, plan.paths[0], "result7x7_with01.pcm");
    }
  }

  GIVEN("MapConfiguration by code with 10x10 map") {
    auto map = yplanner::createMap({{0., 1.}, 0.0}, {10, 10}, {0.1, 0.1});
    const int8_t F = ssr::yplanner::OccupancyGridMap2D::FREE_STATE;
    map->cells = {
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, F, F, F, F, 1, F, F, F, 1, 
      1, F, F, F, F, 1, F, F, F, 1, 
      1, F, F, F, F, 1, F, F, F, 1, 
      1, F, F, F, F, 1, F, F, F, 1, 
      
      1, F, F, F, F, 1, F, F, F, 1, 
      1, F, F, F, F, F, F, F, F, 1, 
      1, F, F, F, F, F, F, F, F, 1, 
      1, F, F, F, F, F, F, F, F, 1, 
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    saveMapAsASCII(map, "temp_map10x10.pgm");

    THEN("Do Plan with robot zero radius") {
      auto plan = ssr::yplanner::planPath(map, {
          {0.25, 0.75, 0}, {0.75, 0.75, 0}, {0.0}
      });
      REQUIRE(plan.status == ssr::yplanner::Plan::PLAN_OK);
      REQUIRE(plan.paths.size() > 0);
      saveMapAndPathAsASCII(map, plan.paths[0], "result10x10.pcm");
    }

    THEN("Do Plan with robot 0.1 radius") {
      auto plan = ssr::yplanner::planPath(map, {
          {0.25, 0.75, 0}, {0.75, 0.75, 0}, {0.1}
      });
      REQUIRE(plan.status == ssr::yplanner::Plan::PLAN_OK);
      REQUIRE(plan.paths.size() > 0);
      saveMapAndPathAsASCII(map, plan.paths[0], "result10x10_with01.pcm");
    }
  }


}