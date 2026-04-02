#include "f_agents_planner.hpp"

#ifdef FLOW_BACKEND_CPLEX

#include <stdexcept>

FAgentsPlanner::Result FAgentsPlanner::solve_flow() {
    throw std::runtime_error("CPLEX backend not implemented yet.");
}

#endif