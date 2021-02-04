#pragma once


namespace ssr {
  namespace yplanner {

    struct Size2D {
    public:
      double width;
      double height;
    };

    struct Point2D {
    public:
      double x;
      double y;
    };

    struct Pose2D {
    public:
      Point2D position;
      double a;
    };
    
    struct GridSize2D {
    public:
      uint32_t width;
      uint32_t height;
    };

    struct GridPoint2D {
    public:
      uint32_t x;
      uint32_t y;
    };

    struct GridPose2D {
    public:
      GridPoint2D position;
      double a;
    };

    
  }
}
