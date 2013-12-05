cmake_minimum_required (VERSION 2.6)
project (FilterDemo)

#--------------------------
# libpointmatcher
find_package(pointmatcher 1.1.0 REQUIRED)
include_directories("${POINTMATCHER_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${pointmatcher_VERSION}")

add_executable(demo FilterDemo.cpp)
#target_link_libraries(demo pointmatcher ${Boost_LIBRARIES} ${NABO_LIBRARY})
target_link_libraries(demo ${POINTMATCHER_LIBRARIES})
