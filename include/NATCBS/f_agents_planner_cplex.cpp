#include "f_agents_planner.hpp"

#ifdef FLOW_BACKEND_CPLEX

#include <algorithm>
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

} 

void FAgentsPlanner::clear_saved_basis() {
    // Reset Warm-Start Basis State 
    // Called when:
    // 1. Network topology changes (can't reuse basis with different structure)
    // 2. Basis injection fails (CPXNETcopybase returns error)
    // 3. Solve fails (can't trust basis from failed solve)
    // 4. On exception (cleanup persistent state)
    // This ensures only valid bases are attempted in next iteration
    saved_arc_basis.clear();
    saved_node_basis.clear();
    saved_basis_arc_count = -1;
    saved_basis_node_count = -1;
}

void FAgentsPlanner::ensure_cplex_model() {
    // Lazy Initialization of Persistent CPLEX Resources
    // This method implements a "warm-start model" pattern:
    // - On first call: create CPLEX environment and network model
    // - On subsequent calls: reuse existing environment and model
    // - Avoids per-iteration allocation/cleanup overhea
    // Create CPLEX environment if not already created
    // The environment is a singleton for this planner instance
    if (cplex_env == nullptr) {
        int status = 0;
        cplex_env = CPXopenCPLEX(&status);
        if (cplex_env == nullptr || status != 0) {
            throw_cplex_error(cplex_env, status, "CPXopenCPLEX");
        }
    }

    if (cplex_net == nullptr) {
        int status = 0;
        cplex_net = CPXNETcreateprob(cplex_env, &status, "wrp_flow_twostage");
        if (cplex_net == nullptr || status != 0) {
            throw_cplex_error(cplex_env, status, "CPXNETcreateprob");
        }
    }
}

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

    std::vector<int> fromnode;
    std::vector<int> tonode;
    std::vector<double> low;
    std::vector<double> up;
    std::vector<double> obj;
    std::vector<double> zero_obj;
    std::vector<double> supply(static_cast<size_t>(num_nodes), 0.0);

    fromnode.reserve(static_cast<size_t>(num_arcs));
    tonode.reserve(static_cast<size_t>(num_arcs));
    low.reserve(static_cast<size_t>(num_arcs));
    up.reserve(static_cast<size_t>(num_arcs));
    obj.reserve(static_cast<size_t>(num_arcs));

    for (ArcIndex a = 0; a < num_arcs; ++a) {
        fromnode.push_back(static_cast<int>(arc_tail[a]));
        tonode.push_back(static_cast<int>(arc_head[a]));
        low.push_back(0.0);
        up.push_back(static_cast<double>(arc_cap[a]));
        obj.push_back(static_cast<double>(arc_cost[a]));
    }

    for (const auto &[from_out_idx, to_in_idx] : commit_edges_d_to_s) {
        const FlowNode &from_out_node = idx_to_node.at(static_cast<size_t>(from_out_idx));
        const FlowNode &to_in_node = idx_to_node.at(static_cast<size_t>(to_in_idx));
        if (from_out_node.type != FlowNode::Type::OUT || to_in_node.type != FlowNode::Type::IN) {
            throw std::runtime_error("CPLEX solve_flow: malformed positive edge commitment.");
        }

        const int time = from_out_node.time;
        const std::pair<Location, Location> edge_pair =
            (from_out_node.location1 < to_in_node.location1)
                ? std::make_pair(from_out_node.location1, to_in_node.location1)
                : std::make_pair(to_in_node.location1, from_out_node.location1);

        const NodeIndex edge_in = node_to_idx.at(
            FlowNode(FlowNode::Type::EDGE_IN, time + 1, edge_pair.first, edge_pair.second)
        );
        const NodeIndex edge_out = node_to_idx.at(
            FlowNode(FlowNode::Type::EDGE_OUT, time + 1, edge_pair.first, edge_pair.second)
        );

        low[static_cast<size_t>(get_arc_idx(from_out_idx, edge_in))] = 1.0;
        low[static_cast<size_t>(get_arc_idx(edge_in, edge_out))] = 1.0;
        low[static_cast<size_t>(get_arc_idx(edge_out, to_in_idx))] = 1.0;
    }

    zero_obj.assign(obj.size(), 0.0);

    const FlowQuantity required_flow = static_cast<FlowQuantity>(num_of_agents);
    supply[static_cast<size_t>(source_idx)] = static_cast<double>(required_flow);
    supply[static_cast<size_t>(sink_idx)] = -static_cast<double>(required_flow);

    ensure_cplex_model();
    int status = 0;

    try {
        const bool topology_unchanged =
            cplex_reuse_model_enabled &&
            prev_node_count == static_cast<int>(num_nodes) &&
            prev_arc_count == static_cast<int>(num_arcs) &&
            prev_fromnode == fromnode &&
            prev_tonode == tonode;

        if (!topology_unchanged) {
            clear_saved_basis();
            status = CPXNETcopynet(
                cplex_env,
                cplex_net,
                CPX_MIN,
                static_cast<int>(num_nodes),
                supply.data(),
                nullptr,
                static_cast<int>(num_arcs),
                fromnode.data(),
                tonode.data(),
                low.data(),
                up.data(),
                zero_obj.data(),
                nullptr
            );
            if (status != 0) {
                throw_cplex_error(cplex_env, status, "CPXNETcopynet");
            }

            prev_node_count = static_cast<int>(num_nodes);
            prev_arc_count = static_cast<int>(num_arcs);
            prev_fromnode = fromnode;
            prev_tonode = tonode;
            if (verbose) {
                std::cout << "[CPLEX] Rebuilt network model (topology changed)." << std::endl;
            }
        } else {
            std::vector<int> arc_idx_2x(static_cast<size_t>(2 * num_arcs), 0);
            std::vector<char> lu(static_cast<size_t>(2 * num_arcs), 'L');
            std::vector<double> bd(static_cast<size_t>(2 * num_arcs), 0.0);
            for (ArcIndex a = 0; a < num_arcs; ++a) {
                const int ia = static_cast<int>(a);
                arc_idx_2x[static_cast<size_t>(a)] = ia;
                lu[static_cast<size_t>(a)] = 'L';
                bd[static_cast<size_t>(a)] = low[static_cast<size_t>(a)];
                arc_idx_2x[static_cast<size_t>(a + num_arcs)] = ia;
                lu[static_cast<size_t>(a + num_arcs)] = 'U';
                bd[static_cast<size_t>(a + num_arcs)] = up[static_cast<size_t>(a)];
            }

            status = CPXNETchgbds(
                cplex_env,
                cplex_net,
                static_cast<int>(2 * num_arcs),
                arc_idx_2x.data(),
                lu.data(),
                bd.data()
            );
            if (status != 0) {
                throw_cplex_error(cplex_env, status, "CPXNETchgbds");
            }

            std::vector<int> arc_idx(static_cast<size_t>(num_arcs), 0);
            for (ArcIndex a = 0; a < num_arcs; ++a) {
                arc_idx[static_cast<size_t>(a)] = static_cast<int>(a);
            }

            status = CPXNETchgobj(
                cplex_env,
                cplex_net,
                static_cast<int>(num_arcs),
                arc_idx.data(),
                zero_obj.data()
            );
            if (status != 0) {
                throw_cplex_error(cplex_env, status, "CPXNETchgobj (zero objective)");
            }

            if (verbose) {
                std::cout << "[CPLEX] Applied differential update (bounds/objective)." << std::endl;
            }
        }

        const bool can_warm_start =
            cplex_warm_start_enabled &&
            saved_basis_arc_count == static_cast<int>(num_arcs) &&
            saved_basis_node_count == static_cast<int>(num_nodes) &&
            static_cast<int>(saved_arc_basis.size()) == saved_basis_arc_count &&
            static_cast<int>(saved_node_basis.size()) == saved_basis_node_count;

        if (can_warm_start) {
            status = CPXNETcopybase(
                cplex_env,
                cplex_net,
                saved_arc_basis.data(),
                saved_node_basis.data()
            );
            if (status != 0) {
                // Basis injection failed, need to discard and fall back to cold-start
                clear_saved_basis();
                if (verbose) {
                    std::cout << "[CPLEX] Warm start basis rejected; fallback to cold start." << std::endl;
                }
            } else if (verbose) {
                std::cout << "[CPLEX] Warm start basis loaded." << std::endl;
            }
        } else if (verbose && cplex_warm_start_enabled) {
            std::cout << "[CPLEX] No reusable basis available; cold start." << std::endl;
        }

        status = CPXNETprimopt(cplex_env, cplex_net);
        if (status != 0) {
            throw_cplex_error(cplex_env, status, "CPXNETprimopt (feasibility stage)");
        }

        const int feas_status = CPXNETgetstat(cplex_env, cplex_net);
        if (feas_status != CPX_STAT_OPTIMAL) {
            clear_saved_basis();
            return INFEASIBLE;
        }

        std::vector<int> arc_index(static_cast<size_t>(num_arcs), 0);
        for (ArcIndex a = 0; a < num_arcs; ++a) {
            arc_index[static_cast<size_t>(a)] = static_cast<int>(a);
        }

        status = CPXNETchgobj(
            cplex_env,
            cplex_net,
            static_cast<int>(num_arcs),
            arc_index.data(),
            obj.data()
        );
        if (status != 0) {
            throw_cplex_error(cplex_env, status, "CPXNETchgobj");
        }

        status = CPXNETprimopt(cplex_env, cplex_net);
        if (status != 0) {
            throw_cplex_error(cplex_env, status, "CPXNETprimopt (min-cost stage)");
        }

        const int opt_status = CPXNETgetstat(cplex_env, cplex_net);
        if (opt_status != CPX_STAT_OPTIMAL) {
            clear_saved_basis();
            return INFEASIBLE;
        }

        double objval = 0.0;
        status = CPXNETsolution(cplex_env, cplex_net, nullptr, &objval, nullptr, nullptr, nullptr, nullptr);
        if (status != 0) {
            throw_cplex_error(cplex_env, status, "CPXNETsolution");
        }

        optimal_cost = static_cast<CostValue>(std::llround(objval));
        maximum_flow = required_flow;

        saved_arc_basis.assign(static_cast<size_t>(num_arcs), 0);
        saved_node_basis.assign(static_cast<size_t>(num_nodes), 0);

        status = CPXNETgetbase(
            cplex_env,
            cplex_net,
            saved_arc_basis.data(),
            saved_node_basis.data()
        );
        if (status == 0) {
            saved_basis_arc_count = static_cast<int>(num_arcs);
            saved_basis_node_count = static_cast<int>(num_nodes);
        } else {
            clear_saved_basis();
        }

        std::vector<double> x(static_cast<size_t>(num_arcs), 0.0);
        status = CPXNETgetx(cplex_env, cplex_net, x.data(), 0, static_cast<int>(num_arcs) - 1);
        if (status != 0) {
            throw_cplex_error(cplex_env, status, "CPXNETgetx");
        }

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

        return SUCCESS;
    } catch (...) {
        if (cplex_net != nullptr) {
            CPXNETfreeprob(cplex_env, &cplex_net);
        }
        if (cplex_env != nullptr) {
            CPXcloseCPLEX(&cplex_env);
        }
        clear_saved_basis();
        prev_fromnode.clear();
        prev_tonode.clear();
        prev_node_count = -1;
        prev_arc_count = -1;
        throw;
    }
}

#endif