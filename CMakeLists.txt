cmake_minimum_required(VERSION 3.5)

project(yplanner LANGUAGES CXX C VERSION ${PROJ_VERSION})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_USE_RELATIVE_PATHS True)

set(PROJ_VERSION 0.0.1)




option(BUILD_YP_TEST "Build yplanner's Test" OFF)


add_subdirectory(thirdparty)
add_subdirectory(include)
add_subdirectory(src)
add_subdirectory(test)
