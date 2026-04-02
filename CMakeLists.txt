cmake_minimum_required(VERSION 3.21)
project(MAWR)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic -gdwarf-4)

set(CMAKE_CXX_FLAGS_RELEASE "-g -O3")
set(CMAKE_CXX_FLAGS_DEBUG   "-g -O0")

set(FLOW_BACKEND "ORTOOLS" CACHE STRING "Flow backend: ORTOOLS or CPLEX")
set_property(CACHE FLOW_BACKEND PROPERTY STRINGS ORTOOLS CPLEX)

find_package(Boost REQUIRED COMPONENTS program_options)

set(MAWR_SOURCES
    main.cpp
    include/common.cpp include/common.hpp
    include/scenario.cpp include/scenario.hpp
    include/MLAStar/mlastar.cpp include/MLAStar/mlastar.hpp
    include/MLAStar/mlastar_node.cpp include/MLAStar/mlastar_node.hpp
    include/AStar/astar.cpp include/AStar/astar.hpp
    include/AStar/astar_node.cpp include/AStar/astar_node.hpp
    include/constraints/constraints.cpp include/constraints/constraints.hpp
    include/constraints/vertex_constraint.cpp include/constraints/vertex_constraint.hpp
    include/constraints/edge_constraint.cpp include/constraints/edge_constraint.hpp
    include/MLAStar/mstate.cpp include/MLAStar/mstate.hpp
    include/MLAStar/mconstraints.cpp include/MLAStar/mconstraints.hpp
    include/NATCBS/natcbs.cpp include/NATCBS/natcbs.hpp
    include/NATCBS/natcbs_node.cpp include/NATCBS/natcbs_node.hpp
    include/NATCBS/realization_conflict.hpp
    include/NATCBS/f_agents_planner.hpp
    include/NATCBS/f_agents_planner_common.cpp
    include/NATCBS/flow_node.cpp include/NATCBS/flow_node.hpp
)

add_executable(MAWR ${MAWR_SOURCES})

target_link_libraries(MAWR
    PRIVATE
    Boost::program_options
)

if(FLOW_BACKEND STREQUAL "ORTOOLS")
    find_package(ortools REQUIRED)

    target_sources(MAWR PRIVATE
        include/NATCBS/f_agents_planner_ortools.cpp
    )

    target_compile_definitions(MAWR PRIVATE FLOW_BACKEND_ORTOOLS)

    target_link_libraries(MAWR
        PRIVATE
        ortools::ortools
    )

elseif(FLOW_BACKEND STREQUAL "CPLEX")
    target_sources(MAWR PRIVATE
        include/NATCBS/f_agents_planner_cplex.cpp
    )

    target_compile_definitions(MAWR PRIVATE FLOW_BACKEND_CPLEX)

    target_include_directories(MAWR PRIVATE
        $ENV{HOME}/cplex2212/cplex/include
    )

    target_link_libraries(MAWR
        PRIVATE
        $ENV{HOME}/cplex2212/cplex/lib/x86-64_linux/static_pic/libcplex.a
        m
        pthread
        dl
    )

else()
    message(FATAL_ERROR "Unsupported FLOW_BACKEND='${FLOW_BACKEND}'. Use ORTOOLS or CPLEX.")
endif()