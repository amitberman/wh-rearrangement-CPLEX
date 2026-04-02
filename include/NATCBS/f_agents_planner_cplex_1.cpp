#include "f_agents_planner.hpp"

#ifdef FLOW_BACKEND_CPLEX

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <iostream>

namespace {

[[noreturn]] void throw_cplex_error(CPXENVptr env, int status, const std::string &where) {
    char errbuf[CPXMESSAGEBUFSIZE];
    if (env != nullptr) {
        CPXgeterrorstring(env, status, errbuf);
        throw std::runtime_error(where + " failed: " + std::string(errbuf));
    }
    throw std::runtime_error(where + " failed with status " + std::to_string(status));
}

} // namespace

FAgentsPlanner::Result FAgentsPlanner::solve_flow() {
    optimal_cost = 0;
    maximum_flow = 0;
    arc_flow.clear();
    flow.clear();

    const NodeIndex num_nodes = static_cast<NodeIndex>(idx_to_node.size());
    const ArcIndex num_arcs = static_cast<ArcIndex>(arc_tail.size());
    if (num_nodes <= 0) {
        throw std::runtime_error("CPLEX solve_flow: graph has no nodes.");
    }

    // The required flow is simply the number of agents.
    const FlowQuantity required_flow = static_cast<FlowQuantity>(num_of_agents);

    std::vector<int> fromnode;
    std::vector<int> tonode;
    std::vector<double> low;
    std::vector<double> up;
    std::vector<double> obj;
    std::vector<double> zero_obj;
    
    // Supply array size is exactly the number of real nodes
    std::vector<double> supply(static_cast<size_t>(num_nodes), 0.0);

    fromnode.reserve(static_cast<size_t>(num_arcs));
    tonode.reserve(static_cast<size_t>(num_arcs));
    low.reserve(static_cast<size_t>(num_arcs));
    up.reserve(static_cast<size_t>(num_arcs));
    obj.reserve(static_cast<size_t>(num_arcs));

    // ---------------------------------------------------------------------
    // NATIVE LOWER-BOUND HANDLING:
    // Iterate through the original arcs. If an arc is marked as "committed",
    // we simply enforce a lower bound of 1.0 directly on it.
    // ---------------------------------------------------------------------
    for (ArcIndex a = 0; a < num_arcs; ++a) {
        NodeIndex tail = arc_tail[a];
        NodeIndex head = arc_head[a];
        
        fromnode.push_back(static_cast<int>(tail));
        tonode.push_back(static_cast<int>(head));
        
        double lower_bound = 0.0;
        
        // commit_edges_d_to_s maps Destination (head) -> Source (tail)
        auto it = commit_edges_d_to_s.find(head);
        if (it != commit_edges_d_to_s.end() && it->second == tail) {
            lower_bound = 1.0; // Force exactly 1 unit through this original arc
        }
        
        low.push_back(lower_bound); 
        up.push_back(static_cast<double>(arc_cap[a])); 
        obj.push_back(static_cast<double>(arc_cost[a])); 
    }

    zero_obj.assign(obj.size(), 0.0);

    // Set supply and demand directly on the original source and sink nodes
    supply[static_cast<size_t>(source_idx)] = static_cast<double>(required_flow);
    supply[static_cast<size_t>(sink_idx)] = -static_cast<double>(required_flow);

    CPXENVptr env = nullptr;
    CPXNETptr net = nullptr;
    int status = 0;

    env = CPXopenCPLEX(&status);
    if (env == nullptr || status != 0) {
        throw_cplex_error(env, status, "CPXopenCPLEX");
    }

    try {
        net = CPXNETcreateprob(env, &status, "wrp_flow_twostage");
        if (net == nullptr || status != 0) {
            throw_cplex_error(env, status, "CPXNETcreateprob");
        }

        // -----------------------------------------------------------------
        // Load the network into CPLEX
        // -----------------------------------------------------------------
        status = CPXNETcopynet(
            env,
            net,
            CPX_MIN,
            static_cast<int>(num_nodes),
            supply.data(),
            nullptr, // node names
            static_cast<int>(num_arcs),
            fromnode.data(),
            tonode.data(),
            low.data(),         // Natively handles the lower bounds here!
            up.data(), 
            zero_obj.data(),    // Stage 1: zero objective (feasibility)
            nullptr             // arc names
        );
        if (status != 0) {
            throw_cplex_error(env, status, "CPXNETcopynet");
        }

        // -----------------------------------------------------------------
        // Stage 1: Feasibility solve
        // -----------------------------------------------------------------
        status = CPXNETprimopt(env, net);
        if (status != 0) {
            throw_cplex_error(env, status, "CPXNETprimopt (feasibility stage)");
        }

        const int feas_status = CPXNETgetstat(env, net);
        if (feas_status != CPX_STAT_OPTIMAL) {
            CPXNETfreeprob(env, &net);
            CPXcloseCPLEX(&env);
            return INFEASIBLE;
        }

        // -----------------------------------------------------------------
        // Stage 2: Restore true costs and solve min-cost flow
        // -----------------------------------------------------------------
        std::vector<int> arc_index(static_cast<size_t>(num_arcs), 0);
        for (ArcIndex a = 0; a < num_arcs; ++a) {
            arc_index[static_cast<size_t>(a)] = static_cast<int>(a);
        }

        status = CPXNETchgobj(
            env,
            net,
            static_cast<int>(num_arcs),
            arc_index.data(),
            obj.data()
        );
        if (status != 0) {
            throw_cplex_error(env, status, "CPXNETchgobj");
        }

        status = CPXNETprimopt(env, net);
        if (status != 0) {
            throw_cplex_error(env, status, "CPXNETprimopt (min-cost stage)");
        }

        const int opt_status = CPXNETgetstat(env, net);
        if (opt_status != CPX_STAT_OPTIMAL) {
            CPXNETfreeprob(env, &net);
            CPXcloseCPLEX(&env);
            return INFEASIBLE;
        }

        double objval = 0.0;
        status = CPXNETsolution(env, net, nullptr, &objval, nullptr, nullptr, nullptr, nullptr);
        if (status != 0) {
            throw_cplex_error(env, status, "CPXNETsolution");
        }

        optimal_cost = static_cast<CostValue>(std::llround(objval));
        maximum_flow = required_flow;

        std::vector<double> x(static_cast<size_t>(num_arcs), 0.0);
        status = CPXNETgetx(env, net, x.data(), 0, static_cast<int>(num_arcs) - 1);
        if (status != 0) {
            throw_cplex_error(env, status, "CPXNETgetx");
        }

        // Extract flow
        arc_flow.assign(static_cast<size_t>(num_arcs), 0);
        flow.clear();

        for (ArcIndex a = 0; a < num_arcs; ++a) {
            const FlowQuantity f = static_cast<FlowQuantity>(
                std::llround(x[static_cast<size_t>(a)])
            );
            arc_flow[static_cast<size_t>(a)] = f;

            if (f > 0) {
                if (verbose) {
                    std::cout << "f "
                              << idx_to_node.at(static_cast<size_t>(arc_tail[a]))
                              << " -> "
                              << idx_to_node.at(static_cast<size_t>(arc_head[a]))
                              << " "
                              << f
                              << std::endl;
                }
                flow[arc_tail[a]] = arc_head[a];
            }
        }

        if (verbose) {
            this->print_flow();
        }

        CPXNETfreeprob(env, &net);
        CPXcloseCPLEX(&env);
        return SUCCESS;
    } catch (...) {
        if (net != nullptr) {
            CPXNETfreeprob(env, &net);
        }
        if (env != nullptr) {
            CPXcloseCPLEX(&env);
        }
        throw;
    }
}

#endif