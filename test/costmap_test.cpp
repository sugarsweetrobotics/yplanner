
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>
#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.hpp>


#include "yplanner/map_2d.h"
using namespace ssr;
using namespace Catch::literals;


SCENARIO( "Map test", "[constmap]" ) {



  GIVEN("Simplest Map") {
    auto map = yplanner::createMap({{0.1, 0.2}, 0.0}, {10, 10}, {0.1, 0.1});
    THEN("Map is valid") {
      REQUIRE(map->config.cellsSize.width == 10);
      REQUIRE(map->config.cellsSize.height == 10);
    }

    THEN("Map GXWX valid") {
      auto p = yplanner::gridToWorld(map->config, {0, 0});
      REQUIRE(p.x == 0.15_a);
      REQUIRE(p.y == 0.15_a);
    }
  }

  GIVEN("Rotated Map coordinate convert test.") {
    auto map = yplanner::createMap({{0.1, 0.2}, M_PI/2}, {10, 10}, {0.1, 0.1});
    THEN("Map is valid") {
      REQUIRE(map->config.cellsSize.width == 10);
      REQUIRE(map->config.cellsSize.height == 10);
    }

    THEN("Map GXWX valid without rotation") {
      auto p = yplanner::gridToWorld(map->config, {0, 0});
      REQUIRE(p.x == 0.15_a);
      REQUIRE(p.y == 0.25_a);
    }

    THEN("Map GXWX validd with rotation") {
      auto p = yplanner::gridToWorld(map->config, {1, 0});
      REQUIRE(p.x == 0.15_a);
      REQUIRE(p.y == 0.35_a);
    }

  }

}


SCENARIO( "Constmap test", "[constmap]" ) {
  
}


