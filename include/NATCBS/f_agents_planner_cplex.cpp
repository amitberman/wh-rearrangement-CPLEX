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

} // namespace

void FAgentsPlanner::clear_saved_basis() {
    // ===== Reset Warm-Start Basis State =====
    // Called when:
    // 1. Network topology changes (can't reuse basis with different structure)
    // 2. Basis injection fails (CPXNETcopybase returns error)
    // 3. Solve fails (can't trust basis from failed solve)
    // 4. On exception (cleanup persistent state)
    // 
    // This ensures only valid bases are attempted in next iteration
    saved_arc_basis.clear();
    saved_node_basis.clear();
    saved_basis_arc_count = -1;
    saved_basis_node_count = -1;
}

void FAgentsPlanner::ensure_cplex_model() {
    // ===== Lazy Initialization of Persistent CPLEX Resources =====
    // This method implements a "warm-start model" pattern:
    // - On first call: create CPLEX environment and network model
    // - On subsequent calls: reuse existing environment and model
    // - Avoids per-iteration allocation/cleanup overhead
    
    // Create CPLEX environment if not already created
    // The environment is a singleton for this planner instance
    if (cplex_env == nullptr) {
        int status = 0;
        cplex_env = CPXopenCPLEX(&status);
        if (cplex_env == nullptr || status != 0) {
            throw_cplex_error(cplex_env, status, "CPXopenCPLEX");
        }
    }

    // Create network model (min-cost flow problem) if not already created
    // This model will be reused (with bounds/costs/basis updates) across iterations
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

    // Build a deterministic ordering for commit-helper arcs.
    // Without this, unordered_map iteration order may vary across iterations,
    // causing false topology-change detections and unnecessary model rebuilds.
    std::vector<std::pair<NodeIndex, NodeIndex>> sorted_commit_edges;
    sorted_commit_edges.reserve(static_cast<size_t>(commit_edges_d_to_s.size()));
    for (const auto &[demand_node, supply_node] : commit_edges_d_to_s) {
        sorted_commit_edges.emplace_back(demand_node, supply_node);
    }
    std::sort(
        sorted_commit_edges.begin(),
        sorted_commit_edges.end(),
        [](const auto &lhs, const auto &rhs) {
            if (lhs.first != rhs.first) {
                return lhs.first < rhs.first;
            }
            return lhs.second < rhs.second;
        }
    );

    // ---------------------------------------------------------------------
    // Commit helper arcs
    //
    // LOWER-BOUND HANDLING:
    // We force exactly 1 unit through each helper arc:
    //      1 <= x <= 1
    // ---------------------------------------------------------------------
    for (const auto &[demand_node, supply_node] : sorted_commit_edges) {
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

    ensure_cplex_model();
    int status = 0;

    try {
        // ===== TOPOLOGY DETECTION: Decide between Rebuild vs Differential Update =====
        // The network topology (arc connectivity) can change across NATCBS iterations
        // when new agents/barriers are discovered. We check if structure is unchanged:
        // - If unchanged: apply fast differential updates (CPXNETchgbds + CPXNETchgobj)
        // - If changed: rebuild entire model via CPXNETcopynet and reset basis
        // 
        // Only possible if MAWR_CPLEX_REUSE_MODEL=1 (persistent model enabled)
        const bool topology_unchanged =
            cplex_reuse_model_enabled &&
            prev_augmented_node_count == static_cast<int>(augmented_num_nodes) &&
            prev_augmented_arc_count == static_cast<int>(augmented_num_arcs) &&
            prev_fromnode == fromnode &&
            prev_tonode == tonode;

        if (!topology_unchanged) {
            // ===== REBUILD PATH: Network topology changed =====
            // Must rebuild the entire model structure because node/arc incidence changed
            // This indicates a significant network change (e.g., new nodes discovered)
            clear_saved_basis();  // Basis invalid for different topology
            
            // Rebuild network via CPXNETcopynet with new structure and zero objectives initially
            status = CPXNETcopynet(
                cplex_env,
                cplex_net,
                CPX_MIN,
                static_cast<int>(augmented_num_nodes),
                supply.data(),
                nullptr,
                static_cast<int>(augmented_num_arcs),
                fromnode.data(),
                tonode.data(),
                low.data(),
                up.data(),
                zero_obj.data(),  // Start with zero objectives; real costs applied later
                nullptr
            );
            if (status != 0) {
                throw_cplex_error(cplex_env, status, "CPXNETcopynet");
            }

            // Update topology fingerprint for next iteration's detection
            prev_augmented_node_count = static_cast<int>(augmented_num_nodes);
            prev_augmented_arc_count = static_cast<int>(augmented_num_arcs);
            prev_fromnode = fromnode;
            prev_tonode = tonode;
            if (verbose) {
                std::cout << "[CPLEX] Rebuilt network model (topology changed)." << std::endl;
            }
        } else {
            // ===== DIFFERENTIAL UPDATE PATH: Network topology unchanged =====
            // Structure is identical to previous iteration; apply fast incremental updates
            // This is a warm-start optimization: reuse model structure, only update data
            
            // Step 1: Update arc bounds (lower and upper bounds via CPXNETchgbds)
            // Prepare pairs of (arc_index, bound_type, new_value) for all arcs
            // Each arc has 2 entries: one for lower bound ('L') and one for upper bound ('U')
            std::vector<int> arc_idx_2x(static_cast<size_t>(2 * augmented_num_arcs), 0);
            std::vector<char> lu(static_cast<size_t>(2 * augmented_num_arcs), 'L');
            std::vector<double> bd(static_cast<size_t>(2 * augmented_num_arcs), 0.0);
            for (ArcIndex a = 0; a < augmented_num_arcs; ++a) {
                const int ia = static_cast<int>(a);
                // Lower bound entry
                arc_idx_2x[static_cast<size_t>(a)] = ia;
                lu[static_cast<size_t>(a)] = 'L';
                bd[static_cast<size_t>(a)] = low[static_cast<size_t>(a)];
                // Upper bound entry (offset by augmented_num_arcs)
                arc_idx_2x[static_cast<size_t>(a + augmented_num_arcs)] = ia;
                lu[static_cast<size_t>(a + augmented_num_arcs)] = 'U';
                bd[static_cast<size_t>(a + augmented_num_arcs)] = up[static_cast<size_t>(a)];
            }

            // Apply all bound changes in one call
            status = CPXNETchgbds(
                cplex_env,
                cplex_net,
                static_cast<int>(2 * augmented_num_arcs),
                arc_idx_2x.data(),
                lu.data(),
                bd.data()
            );
            if (status != 0) {
                throw_cplex_error(cplex_env, status, "CPXNETchgbds");
            }

            // Step 2: Update arc objectives (costs via CPXNETchgobj)
            // Currently setting all to zero; will be updated to real costs before solving
            std::vector<int> arc_idx(static_cast<size_t>(augmented_num_arcs), 0);
            for (ArcIndex a = 0; a < augmented_num_arcs; ++a) {
                arc_idx[static_cast<size_t>(a)] = static_cast<int>(a);
            }

            status = CPXNETchgobj(
                cplex_env,
                cplex_net,
                static_cast<int>(augmented_num_arcs),
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

        // ===== WARM-START BASIS INJECTION =====
        // If enabled (MAWR_CPLEX_WARM_START=1) and compatible basis available,
        // inject saved basis before solving. This gives simplex a head-start.
        // Basis validity check:
        // 1. cplex_warm_start_enabled flag is true
        // 2. Saved basis dimensions match current network
        // 3. Basis arrays are non-empty
        const bool can_warm_start =
            cplex_warm_start_enabled &&
            saved_basis_arc_count == static_cast<int>(augmented_num_arcs) &&
            saved_basis_node_count == static_cast<int>(augmented_num_nodes) &&
            static_cast<int>(saved_arc_basis.size()) == saved_basis_arc_count &&
            static_cast<int>(saved_node_basis.size()) == saved_basis_node_count;

        if (can_warm_start) {
            // Inject saved basis: CPXNETcopybase loads arc and node status arrays
            // These contain CPX status codes (basis: lower/upper/free/basic)
            status = CPXNETcopybase(
                cplex_env,
                cplex_net,
                saved_arc_basis.data(),
                saved_node_basis.data()
            );
            if (status != 0) {
                // Basis injection failed; discard and fall back to cold-start
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

        // -----------------------------------------------------------------
        // Stage 1: Feasibility solve
        // -----------------------------------------------------------------
        status = CPXNETprimopt(cplex_env, cplex_net);
        if (status != 0) {
            throw_cplex_error(cplex_env, status, "CPXNETprimopt (feasibility stage)");
        }

        const int feas_status = CPXNETgetstat(cplex_env, cplex_net);
        if (feas_status != CPX_STAT_OPTIMAL) {
            clear_saved_basis();
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
            cplex_env,
            cplex_net,
            static_cast<int>(augmented_num_arcs),
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

        // ===== SAVE SIMPLEX BASIS FOR NEXT ITERATION =====
        // After successful min-cost solve, extract and save the optimal basis
        // This basis will be reused in next NATCBS iteration if topology unchanged
        // Basis consists of:
        // - saved_arc_basis[a]: status code for each arc (tells if at lower/upper bound, or basic)
        // - saved_node_basis[n]: status code for each node (free, at lower, at upper, or basic)
        
        saved_arc_basis.assign(static_cast<size_t>(augmented_num_arcs), 0);
        saved_node_basis.assign(static_cast<size_t>(augmented_num_nodes), 0);
        
        // Extract basis from solved model
        status = CPXNETgetbase(
            cplex_env,
            cplex_net,
            saved_arc_basis.data(),
            saved_node_basis.data()
        );
        if (status == 0) {
            // Basis extraction succeeded; save dimensions for validation check in next iteration
            saved_basis_arc_count = static_cast<int>(augmented_num_arcs);
            saved_basis_node_count = static_cast<int>(augmented_num_nodes);
        } else {
            // Basis extraction failed; discard invalid basis
            clear_saved_basis();
        }

        std::vector<double> x(static_cast<size_t>(augmented_num_arcs), 0.0);
        status = CPXNETgetx(cplex_env, cplex_net, x.data(), 0, static_cast<int>(augmented_num_arcs) - 1);
        if (status != 0) {
            throw_cplex_error(cplex_env, status, "CPXNETgetx");
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
        prev_augmented_node_count = -1;
        prev_augmented_arc_count = -1;
        throw;
    }
}

#endif