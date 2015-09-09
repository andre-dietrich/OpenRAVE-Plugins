cmake_minimum_required (VERSION 2.6.0)
project(filter)
find_package(OpenRAVE REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(filter SHARED src/filter.cpp)
set_target_properties(filter PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(filter ${OpenRAVE_LIBRARIES})
#install(TARGETS filter DESTINATION .)

execute_process(COMMAND "openrave-config" "--share-dir" OUTPUT_VARIABLE SHARE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
set(PLUGIN_DIR ${SHARE_DIR}/plugins)
message(STATUS "Plugin directory: ${PLUGIN_DIR}")
install(TARGETS filter DESTINATION ${PLUGIN_DIR})