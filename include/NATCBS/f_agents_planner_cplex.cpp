#include "f_agents_planner.hpp"

#ifdef FLOW_BACKEND_CPLEX

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

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

    // Same augmented network structure as the OR-Tools path.
    const ArcIndex commit_arc_count = static_cast<ArcIndex>(commit_edges_d_to_s.size());
    const ArcIndex augmented_num_arcs = num_arcs + 2 * commit_arc_count + 2;
    const NodeIndex total_source = num_nodes;
    const NodeIndex total_sink = num_nodes + 1;
    const NodeIndex augmented_num_nodes = num_nodes + 2;

    const FlowQuantity required_flow =
        static_cast<FlowQuantity>(num_of_agents + static_cast<int>(commit_edges_d_to_s.size()));

    std::vector<int> fromnode;
    std::vector<int> tonode;
    std::vector<double> low;
    std::vector<double> up;
    std::vector<double> obj;
    std::vector<double> zero_obj;
    std::vector<double> supply(static_cast<size_t>(augmented_num_nodes), 0.0);

    fromnode.reserve(static_cast<size_t>(augmented_num_arcs));
    tonode.reserve(static_cast<size_t>(augmented_num_arcs));
    low.reserve(static_cast<size_t>(augmented_num_arcs));
    up.reserve(static_cast<size_t>(augmented_num_arcs));
    obj.reserve(static_cast<size_t>(augmented_num_arcs));

    // ---------------------------------------------------------------------
    // Original planner arcs
    //
    // LOWER-BOUND HANDLING:
    // CPLEX takes lower bounds directly through low[] in CPXNETcopynet(...).
    // For original planner arcs, lower bound is currently 0 and upper bound
    // is arc_cap[a].
    // ---------------------------------------------------------------------
    for (ArcIndex a = 0; a < num_arcs; ++a) {
        fromnode.push_back(static_cast<int>(arc_tail[a]));
        tonode.push_back(static_cast<int>(arc_head[a]));
        low.push_back(0.0);                                  // lower bound
        up.push_back(static_cast<double>(arc_cap[a]));      // upper bound
        obj.push_back(static_cast<double>(arc_cost[a]));    // real optimization cost
    }

    // ---------------------------------------------------------------------
    // Commit helper arcs
    //
    // LOWER-BOUND HANDLING:
    // We force exactly 1 unit through each helper arc:
    //      1 <= x <= 1
    // ---------------------------------------------------------------------
    for (const auto &[demand_node, supply_node] : commit_edges_d_to_s) {
        fromnode.push_back(static_cast<int>(total_source));
        tonode.push_back(static_cast<int>(supply_node));
        low.push_back(1.0);                                 // lower bound
        up.push_back(1.0);                                  // upper bound
        obj.push_back(0.0);

        fromnode.push_back(static_cast<int>(demand_node));
        tonode.push_back(static_cast<int>(total_sink));
        low.push_back(1.0);                                 // lower bound
        up.push_back(1.0);                                  // upper bound
        obj.push_back(0.0);
    }

    // ---------------------------------------------------------------------
    // Global source/sink coupling arcs
    //
    // LOWER-BOUND HANDLING:
    // We force exactly num_of_agents units through these arcs:
    //      num_of_agents <= x <= num_of_agents
    // ---------------------------------------------------------------------
    fromnode.push_back(static_cast<int>(total_source));
    tonode.push_back(static_cast<int>(source_idx));
    low.push_back(static_cast<double>(num_of_agents));      // lower bound
    up.push_back(static_cast<double>(num_of_agents));       // upper bound
    obj.push_back(0.0);

    fromnode.push_back(static_cast<int>(sink_idx));
    tonode.push_back(static_cast<int>(total_sink));
    low.push_back(static_cast<double>(num_of_agents));      // lower bound
    up.push_back(static_cast<double>(num_of_agents));       // upper bound
    obj.push_back(0.0);

    zero_obj.assign(obj.size(), 0.0);

    // Node supplies on augmented network.
    supply[static_cast<size_t>(total_source)] = static_cast<double>(required_flow);
    supply[static_cast<size_t>(total_sink)] = -static_cast<double>(required_flow);

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
        // LOWER-BOUND HANDLING:
        // -----------------------------------------------------------------
        status = CPXNETcopynet(
            env,
            net,
            CPX_MIN,
            static_cast<int>(augmented_num_nodes),
            supply.data(),
            nullptr, // node names
            static_cast<int>(augmented_num_arcs),
            fromnode.data(),
            tonode.data(),
            low.data(),         // lower bounds used here
            up.data(),          // upper bounds used here
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
        std::vector<int> arc_index(static_cast<size_t>(augmented_num_arcs), 0);
        for (ArcIndex a = 0; a < augmented_num_arcs; ++a) {
            arc_index[static_cast<size_t>(a)] = static_cast<int>(a);
        }

        status = CPXNETchgobj(
            env,
            net,
            static_cast<int>(augmented_num_arcs),
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

        std::vector<double> x(static_cast<size_t>(augmented_num_arcs), 0.0);
        status = CPXNETgetx(env, net, x.data(), 0, static_cast<int>(augmented_num_arcs) - 1);
        if (status != 0) {
            throw_cplex_error(env, status, "CPXNETgetx");
        }

        // Only original planner arcs are needed downstream.
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