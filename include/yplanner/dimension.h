#pragma once

#include <cmath>
#include <cstdint>

namespace ssr {
  namespace yplanner {


    struct Time {
      unsigned long sec;
      unsigned long nsec;
    };

    struct Vector2D {
    public:
      double v[2];

      Vector2D(): v{0,0} {}
      Vector2D(const double x, const double y) : v{x, y} {}
    };

    inline double norm(const Vector2D& v) {
      return sqrt(v.v[0]*v.v[0] + v.v[1]*v.v[1]);
    }

    struct Size2D {
    public:
      double width;
      double height;

      Size2D(double w, double h) : width(w), height(h) {}
    };

    struct Point2D {
    public:
      double x;
      double y;

      Point2D(const double _x=0, const double _y=0): x(_x), y(_y) {}
    };

    struct Pose2D {
    public:
      Point2D position;
      double a;

      Pose2D(double _x=0, double _y=0, double _a=0): position(_x, _y), a(_a) {}
      Pose2D(const Point2D& pos, double _a): position(pos), a(_a) {}
    };

    struct Velocity2D {
    public:
      double vx;
      double vy;
      double va;

      Velocity2D(): vx(0), vy(0), va(0) {}
      Velocity2D(const double x, const double y, const double a): vx(x), vy(y), va(a) {}

    };
    
    struct GridSize2D {
    public:
      uint32_t width;
      uint32_t height;

      GridSize2D(uint32_t w, uint32_t h): width(w), height(h) {}
    };

    struct GridPoint2D {
    public:
      uint32_t x;
      uint32_t y;

      GridPoint2D(uint32_t _x=0, uint32_t _y=0) : x(_x), y(_y) {}
    };

    inline bool operator==(const GridPoint2D& x0, const GridPoint2D& x1) {
      return x0.x == x1.x && x0.y == x1.y;
    }

    inline bool operator!=(const GridPoint2D& x0, const GridPoint2D& x1) {
      return !(x0 == x1);
    }

    struct GridVector2D {
    public:
      int64_t v[2];

      GridVector2D(): v{0,0} {}
      GridVector2D(const int64_t x, const int64_t y) : v{x, y} {}
    };

    inline GridVector2D operator-(const GridPoint2D& x0, const GridPoint2D& x1) {
      return {x0.x - x1.x, x0.y - x1.y};
    }

    struct GridPose2D {
    public:
      GridPoint2D position;
      double a;

      GridPose2D(uint32_t _x=0, uint32_t _y=0, double _a=0): position(_x, _y), a(_a) {}
      GridPose2D(const GridPoint2D& pos, double _a): position(pos), a(_a) {}
    };

    struct RobotGeometry {
      float robotRadius;

      float getRobotRadius() { return robotRadius; }
    };


  }
}
