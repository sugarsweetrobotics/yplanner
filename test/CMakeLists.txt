# Test

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/thirdparty/Catch2/single_include/catch2)

set(LIBS yplanner)

function(declare_test arg1)
  add_executable(${arg1}_test ${arg1}_test.cpp)
  target_link_libraries(${arg1}_test ${PROJECT_NAME} Catch2::Catch2)
  add_dependencies(${arg1}_test ${PROJECT_NAME} ${EXAMPLE_MODULES})
  add_test(NAME ${arg1}_test COMMAND $<TARGET_FILE:${arg1}_test>
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test
  )
endfunction()

declare_test(costmap)


