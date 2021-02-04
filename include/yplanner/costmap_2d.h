#pragma once

#include <vector>
#include "yplanner/map.h"
namespace ssr {
  namespace yplanner {
    
    struct CostMap2D {
    public:
      MapConfig2D config;
      std::vector<float> cells;
    };

    
  }
}
