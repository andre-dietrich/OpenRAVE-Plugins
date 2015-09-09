cmake_minimum_required (VERSION 2.6.0)
project(distancesensor)

find_package(OpenRAVE REQUIRED)

message(STATUS ${OpenRAVE_Boost_VERSION})

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(distancesensor SHARED src/distancesensor.cpp)
set_target_properties(distancesensor PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(distancesensor ${OpenRAVE_LIBRARIES})
#install(TARGETS distancesensor DESTINATION .)

execute_process(COMMAND "openrave-config" "--share-dir" OUTPUT_VARIABLE SHARE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "Plugin directory: ${SHARE_DIR}/plugins")
install(TARGETS distancesensor DESTINATION ${SHARE_DIR}/plugins)

install(DIRECTORY beams DESTINATION ${SHARE_DIR}/models)