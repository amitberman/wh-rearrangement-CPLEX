cmake_minimum_required(VERSION 3.21)
project(MAWR)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -gdwarf-4")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

find_package(Boost CONFIG COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})

include_directories("/home/crl05/gurobi1201/linux64/include/")

find_package(ortools REQUIRED)
include_directories(${ORTOOLS_INCLUDE_DIRS})

add_executable(MAWR main.cpp
        include/common.cpp include/common.hpp
        include/scenario.cpp include/scenario.hpp
        include/MLAStar/mlastar.cpp include/MLAStar/mlastar.hpp include/MLAStar/mlastar_node.cpp include/MLAStar/mlastar_node.hpp
        include/AStar/astar.cpp include/AStar/astar.hpp include/AStar/astar_node.cpp include/AStar/astar_node.hpp
        include/constraints/constraints.cpp include/constraints/constraints.hpp include/constraints/vertex_constraint.cpp include/constraints/vertex_constraint.hpp include/constraints/edge_constraint.cpp include/constraints/edge_constraint.hpp
        include/MLAStar/mstate.cpp include/MLAStar/mstate.hpp include/MLAStar/mconstraints.cpp include/MLAStar/mconstraints.hpp
        include/NATCBS/natcbs.cpp include/NATCBS/natcbs.hpp include/NATCBS/natcbs_node.cpp include/NATCBS/natcbs_node.hpp include/NATCBS/realization_conflict.hpp
        include/NATCBS/f_agents_planner.cpp include/NATCBS/f_agents_planner.hpp
        include/NATCBS/flow_node.cpp include/NATCBS/flow_node.hpp)

target_link_libraries(MAWR ${Boost_LIBRARIES})
target_link_libraries(MAWR ortools::ortools)

