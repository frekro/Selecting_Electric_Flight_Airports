cmake_minimum_required(VERSION 3.5)

project(facilitylocation)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

enable_testing()

find_package(SCIP REQUIRED)

include_directories(${SCIP_INCLUDE_DIRS})

find_library(SOPLEX_LIBRARY soplexshared)
find_library(GMP_LIBRARY gmp)

find_package(GUROBI REQUIRED)

include_directories(${GUROBI_INCLUDE_DIRS})

find_package(Boost 1.54 COMPONENTS date_time filesystem log log_setup program_options thread system REQUIRED)

add_definitions(-DBOOST_LOG_DYN_LINK)

include_directories(${BOOST_INCLUDE_DIRS})

find_package(GRAPHVIZ)

if(GRAPHVIZ_FOUND)
  include_directories(${GRAPHVIZ_INCLUDE_DIRS})
else()
  message (STATUS "Compilation without GRAPHVIZ. No graph printing possible!")
endif()

add_compile_definitions(PROG_WITH_GRAPHVIZ=${GRAPHVIZ_FOUND})

if(CMAKE_BUILD_TYPE MATCHES "Release")
  message(STATUS "Release mode, disabling assertions")
  add_definitions("-DNDEBUG")
endif()

include_directories(${CMAKE_BINARY_DIR}/)

add_subdirectory(src)
