#include "f_agents_planner.hpp"
#include "ortools/graph/max_flow.h"


FAgentsPlanner::FAgentsPlanner(int map_rows, int map_cols, const vector<vector<CellType>> &map,
                         vector<Location> &agent_start_locations, int num_of_tasks, bool verbose) :
        map_rows(map_rows), map_cols(map_cols), map(map), num_of_agents(static_cast<int>(agent_start_locations.size())),
        agent_start_locations(agent_start_locations), num_of_tasks(num_of_tasks), verbose(verbose),
        source_node(FlowNode::SOURCE, -1), source_idx(get_node_idx(source_node)),
        sink_node(FlowNode::SINK, -1), sink_idx(get_node_idx(sink_node)),
        current_makespan(0), max_makespan(0), num_edges_before_sink(0),
        optimal_cost(0), maximum_flow(0),
        agent_paths(num_of_agents, shared_ptr<TimedPath>()), realization_conflict(nullptr) {
    reached_locations.reserve(map_rows * map_cols);
    map_edges.reserve(map_rows * map_cols * 2 - map_rows - map_cols);
    new_reached_locations.reserve(num_of_agents*4);
    for (int i = 0; i < num_of_agents; ++i) {
        NodeIndex idx = get_node_idx(FlowNode(FlowNode::Type::OUT, 0, agent_start_locations[i]));
        add_arc(source_idx, idx, NO_COST);
        reached_locations.emplace_back(agent_start_locations[i]);
        new_reached_locations.insert(agent_start_locations[i]);
    }
}


FAgentsPlanner::NodeIndex FAgentsPlanner::get_node_idx(const FlowNode &node) {
    auto it = node_to_idx.find(node);
    if (it == node_to_idx.end()) {
        const auto idx = static_cast<NodeIndex>(idx_to_node.size());
        idx_to_node.push_back(node);
        node_to_idx.emplace(node, idx);
        return idx;
    }
    return it->second;
}

FAgentsPlanner::ArcIndex FAgentsPlanner::get_arc_idx(NodeIndex tail, NodeIndex head) {
    return arc_map.at(tail).at(head);
}

void FAgentsPlanner::add_arc(NodeIndex tail, NodeIndex head, CostValue cost) {
    const auto idx = static_cast<ArcIndex>(arc_tail.size());
    arc_map[tail][head] = idx;
    arc_tail.push_back(tail);
    arc_head.push_back(head);
    arc_cost.push_back(cost);
    arc_cap.push_back(CAP);
    num_edges_before_sink++;
}

void FAgentsPlanner::add_arc_to_sink(NodeIndex tail) {
    arc_tail.push_back(tail);
    arc_head.push_back(sink_idx);
    arc_cost.push_back(NO_COST);
    arc_cap.push_back(CAP);
}


void FAgentsPlanner::update_map_edges() {
    if (reached_all) {
        return;
    }

    reached_all = true;

    bool b2;
    unordered_set<Location> next_reached_locations(new_reached_locations.size() * 4);
    unordered_set<Location> done_new(new_reached_locations.size());
    for (const Location& loc: new_reached_locations) {
        for (int i = 0; i < 4; i++) {
            Location new_loc(loc.x + dx[i], loc.y + dy[i]);
            if (new_loc.x < 0 || new_loc.x >= map_rows || new_loc.y < 0 || new_loc.y >= map_cols ||
                map[new_loc.x][new_loc.y] == CellType::BLOCKED) {
                continue;
            }
            std::pair edge = (loc < new_loc) ? std::make_pair(loc, new_loc) : std::make_pair(new_loc, loc);

            if (recently_reached_locations.count(new_loc) || done_new.count(new_loc)) {
                assert(std::any_of(map_edges.begin(), map_edges.end(),
                                   [edge](const std::pair<Location, Location> &e) { return e == edge; }));
                assert(std::any_of(reached_locations.begin(), reached_locations.end(),
                                   [new_loc](const Location loc) { return loc == new_loc; }));
                continue;
            }
            if (next_reached_locations.count(new_loc) || new_reached_locations.count(new_loc)) {
                assert(std::all_of(map_edges.begin(), map_edges.end(),
                                   [edge](const std::pair<Location, Location> &e) { return e != edge; }));
                assert(std::any_of(reached_locations.begin(), reached_locations.end(),
                                   [new_loc](const Location loc) { return loc == new_loc; }));
                map_edges.emplace_back(edge);
                continue;
            }

            assert(std::all_of(map_edges.begin(), map_edges.end(),
                               [edge](const std::pair<Location, Location> &e) { return e != edge; }));
            map_edges.emplace_back(edge);
            std::tie(std::ignore, b2) = next_reached_locations.emplace(new_loc);

            assert(b2);
            reached_all = false;
            assert(std::all_of(reached_locations.begin(), reached_locations.end(),
                               [new_loc](const Location loc) { return loc != new_loc; }));
            reached_locations.emplace_back(new_loc);
        }
        done_new.emplace(loc);
    }

    if (reached_all) {
        reached_all = true;
        recently_reached_locations.clear();
        new_reached_locations.clear();
        return;
    }

    recently_reached_locations = std::move(new_reached_locations);
    new_reached_locations = std::move(next_reached_locations);
}

void FAgentsPlanner::update_graph(int makespan) {
    if (current_makespan == makespan) {
        return;
    } else if (verbose) {
        std::cout << "Updating graph from makespan " << current_makespan << " to " << makespan << std::endl;
    }

    // remove all edges from previous makespan to sink
    arc_tail.resize(num_edges_before_sink);
    arc_head.resize(num_edges_before_sink);
    arc_cost.resize(num_edges_before_sink);
    arc_cap.resize(num_edges_before_sink);

    for (int t = max_makespan + 1; t <= makespan; t++) {
        // v_t-1_out -> v_t_in
        for (auto &loc : reached_locations) {
            NodeIndex v_out = node_to_idx.at(FlowNode(FlowNode::Type::OUT, t - 1, loc));
            NodeIndex v_in = get_node_idx(FlowNode(FlowNode::Type::IN, t, loc));
            this->add_arc(v_out, v_in, UNIT_COST);
        }

        // edge gadgets
        update_map_edges();
        for (const auto& [loc1, loc2] : map_edges) {
            auto v_1_out_it = node_to_idx.find(FlowNode(FlowNode::Type::OUT, t - 1, loc1));
            auto v_2_out_it = node_to_idx.find(FlowNode(FlowNode::Type::OUT, t - 1, loc2));

            NodeIndex edge_in = get_node_idx(FlowNode(FlowNode::Type::EDGE_IN, t, loc1, loc2));

            assert(v_1_out_it != node_to_idx.end() || v_2_out_it != node_to_idx.end());
            if (v_1_out_it != node_to_idx.end()) {
                add_arc(v_1_out_it->second, edge_in, NO_COST);
            }
            if (v_2_out_it != node_to_idx.end()) {
                add_arc(v_2_out_it->second, edge_in, NO_COST);
            }

            NodeIndex edge_out = get_node_idx(FlowNode(FlowNode::Type::EDGE_OUT, t, loc1, loc2));
            this->add_arc(edge_in, edge_out, UNIT_COST);

            NodeIndex v_1_in = get_node_idx(FlowNode(FlowNode::Type::IN, t, loc1));
            NodeIndex v_2_in = get_node_idx(FlowNode(FlowNode::Type::IN, t, loc2));

            this->add_arc(edge_out, v_1_in, NO_COST);
            this->add_arc(edge_out, v_2_in, NO_COST);
        }

        // v_t_in -> v_t_out
        for (auto &loc : reached_locations) {
            NodeIndex v_in = node_to_idx.at(FlowNode(FlowNode::Type::IN, t, loc));
            NodeIndex v_out = get_node_idx(FlowNode(FlowNode::Type::OUT, t, loc));
            this->add_arc(v_in, v_out, NO_COST);
        }
    }

    // v_T_out -> sink
    for (auto &loc : reached_locations) {
        FlowNode f_node(FlowNode::Type::OUT, makespan, loc);
        auto it = node_to_idx.find(f_node);
        if (it == node_to_idx.end()) continue;
        this->add_arc_to_sink(it->second);
    }

    current_makespan = makespan;
    max_makespan = std::max(max_makespan, makespan);
}

void FAgentsPlanner::update_graph_edge_costs(const vector<shared_ptr<TimedPath>> &obs_paths) {
    for (auto &[e, c]: orig_edge_costs) {
        arc_cost[e] = c;
    }
    orig_edge_costs.clear();

    for (auto &path : obs_paths) {
        int last_t = std::min(current_makespan, path->back().time);
        for (int t = path->front().time; t < last_t; t++) {
            Location loc1 = path->get_location_at_time(t);
            Location loc2 = path->get_location_at_time(t + 1);
            if (loc1 == loc2) {
                continue;
            }
            std::pair edge = (loc1 < loc2) ? std::make_pair(loc1, loc2) : std::make_pair(loc2, loc1);

            NodeIndex edge_in = node_to_idx.at(FlowNode(FlowNode::Type::EDGE_IN, t + 1, edge.first, edge.second));
            NodeIndex edge_out = node_to_idx.at(FlowNode(FlowNode::Type::EDGE_OUT, t + 1, edge.first, edge.second));
            update_single_edge_cost(edge_in, edge_out, NO_COST);

            auto v2_out_it = node_to_idx.find(FlowNode(FlowNode::Type::OUT, t, loc2));
            if (v2_out_it == node_to_idx.end()) {
                continue;
            }
            NodeIndex v1_in = node_to_idx.at(FlowNode(FlowNode::Type::IN, t + 1, loc1));

            update_single_edge_cost(v2_out_it->second, edge_in, UNIT_COST);
            update_single_edge_cost(edge_out, v1_in, UNIT_COST);
        }
    }
}

void FAgentsPlanner::update_single_edge_cost(NodeIndex tail, NodeIndex head, CostValue cost) {
    ArcIndex arc = get_arc_idx(tail, head);
    orig_edge_costs.emplace(arc, arc_cost[arc]);
    arc_cost[arc] = cost;
}

void FAgentsPlanner::update_sources_and_sinks(const vector<shared_ptr<Constraints>> &constraints_table) {
    commit_edges_d_to_s.clear();
    for (auto &e : deleted_arcs){
        arc_cap[e] = CAP;
    }
    deleted_arcs.clear();

    for (auto &constraints: constraints_table) {
        for (auto &[t, edge] : constraints->get_positive_edge_constraints()) {
            if (t >= current_makespan) continue;
            NodeIndex v_from_out = node_to_idx.at(FlowNode(FlowNode::Type::OUT, t, edge.first));
            NodeIndex v_to_in = node_to_idx.at(FlowNode(FlowNode::Type::IN, t + 1, edge.second));
            commit_edges_d_to_s.emplace(v_from_out, v_to_in);

            std::pair edge_pair = (edge.first < edge.second) ? edge : std::make_pair(edge.second, edge.first);
            ArcIndex arc = get_arc_idx(node_to_idx.at(FlowNode(FlowNode::Type::EDGE_IN, t + 1, edge_pair.first, edge_pair.second)),
                                       node_to_idx.at(FlowNode(FlowNode::Type::EDGE_OUT, t + 1, edge_pair.first, edge_pair.second)));
            arc_cap[arc] = 0;
            deleted_arcs.emplace(arc);
        }
    }
}

FAgentsPlanner::Result FAgentsPlanner::find_paths(const vector<shared_ptr<TimedPath>> &obs_paths,
                                            const vector<shared_ptr<Constraints>> &constraints_table, int makespan) {
    update_graph(makespan);
    update_graph_edge_costs(obs_paths);
    update_sources_and_sinks(constraints_table);

    auto result = this->solve_flow();
    if (result != SUCCESS) {
        return result;
    }

    return this->extract_agent_paths_and_detect_conflict(obs_paths);
}

// ORTOOLS_MIGRATION: Direct OR-Tools dependency (GenericMaxFlow + GenericMinCostFlow).
FAgentsPlanner::Result FAgentsPlanner::solve_flow() {
    optimal_cost = 0;
    maximum_flow = 0;
    arc_flow.clear(); flow.clear();
    const auto num_nodes = static_cast<NodeIndex>(idx_to_node.size());
    const auto num_arcs = static_cast<ArcIndex>(arc_tail.size());
    assert(num_nodes > 0);

    // Feasibility checking, and possible supply/demand adjustment, is done by:
    // 1. Creating a new source and sink node.
    // 2. Taking all nodes that have a non-zero supply or demand and connecting them to the source or sink
    //    respectively. The arc thus added has a capacity of the supply or demand.
    // 3. Computing the max flow between the new source and sink.
    // 4. Checking that the max flow is equal to the total supply/demand (and returning INFEASIBLE if it isn't).
    // 5. Running min-cost_type max-flow on this augmented graph, using the max flow computed in step 3 as the supply of
    //    the source and demand of the sink.
    const ArcIndex augmented_num_arcs = num_arcs + 2*static_cast<ArcIndex>(commit_edges_d_to_s.size()) + 2;
    const NodeIndex total_source = num_nodes;
    const NodeIndex total_sink = num_nodes + 1;
    const NodeIndex augmented_num_nodes = num_nodes + 2;

    Graph graph(augmented_num_nodes, augmented_num_arcs);

    for (ArcIndex arc = 0; arc < num_arcs; ++arc) {
        graph.AddArc(arc_tail.at(arc), arc_head.at(arc));
    }

    for (auto &[demand_node, supply_node] : commit_edges_d_to_s) {
        graph.AddArc(total_source, supply_node);
        graph.AddArc(demand_node, total_sink);
    }

    graph.AddArc(total_source, source_idx);
    graph.AddArc(sink_idx, total_sink);

    graph.Build(&arc_permutation);

    {
        operations_research::GenericMaxFlow<Graph> max_flow(&graph, total_source, total_sink);
        ArcIndex arc;
        for (arc = 0; arc < num_arcs; ++arc) {
            max_flow.SetArcCapacity(PermutedArc(arc), arc_cap.at(arc));
        }
        for (; arc < augmented_num_arcs - 2; ++arc) {
            max_flow.SetArcCapacity(PermutedArc(arc), CAP);
        }

        max_flow.SetArcCapacity(PermutedArc(arc++), num_of_agents);
        max_flow.SetArcCapacity(PermutedArc(arc++), num_of_agents);

        assert(arc == augmented_num_arcs);
        if (!max_flow.Solve()) {
            throw std::runtime_error("Max flow could not be computed.");
        }
        maximum_flow = static_cast<FlowQuantity>(max_flow.GetOptimalFlow());
    }

    if (maximum_flow != (num_of_agents + static_cast<ArcIndex>(commit_edges_d_to_s.size()))) {
        return INFEASIBLE;
    }

    MinCostFlow min_cost_flow(&graph);
    ArcIndex arc;
    for (arc = 0; arc < num_arcs; ++arc) {
        ArcIndex permuted_arc = PermutedArc(arc);
        min_cost_flow.SetArcUnitCost(permuted_arc, arc_cost.at(arc));
        min_cost_flow.SetArcCapacity(permuted_arc, arc_cap.at(arc));
    }

    for(; arc < augmented_num_arcs - 2; ++arc) {
        ArcIndex permuted_arc = PermutedArc(arc);
        min_cost_flow.SetArcUnitCost(permuted_arc, NO_COST);
        min_cost_flow.SetArcCapacity(permuted_arc, CAP);
    }

    ArcIndex permuted_arc = PermutedArc(arc++);
    min_cost_flow.SetArcUnitCost(permuted_arc, NO_COST);
    min_cost_flow.SetArcCapacity(permuted_arc, num_of_agents);
    permuted_arc = PermutedArc(arc);
    min_cost_flow.SetArcUnitCost(permuted_arc, NO_COST);
    min_cost_flow.SetArcCapacity(permuted_arc, num_of_agents);

    min_cost_flow.SetNodeSupply(total_source, maximum_flow);
    min_cost_flow.SetNodeSupply(total_sink, -maximum_flow);
    min_cost_flow.SetCheckFeasibility(false);

    arc_flow.resize(num_arcs);
    flow.clear();
    if (min_cost_flow.Solve()) {
        optimal_cost = static_cast<CostValue>(min_cost_flow.GetOptimalCost());
        for (arc = 0; arc < num_arcs; ++arc) {
            arc_flow[arc] = static_cast<FlowQuantity>(min_cost_flow.Flow(PermutedArc(arc)));
            if (arc_flow[arc] > 0) {
                if (verbose){
                    std::cout << "f " << idx_to_node.at(arc_tail.at(arc)) << " -> " << idx_to_node.at(arc_head.at(arc)) << " " << arc_flow[arc] << std::endl;
                }
                flow[arc_tail.at(arc)] = arc_head.at(arc);
            }
        }
    }
    auto status = min_cost_flow.status();
    assert(status == operations_research::MinCostFlow::OPTIMAL);

    if (verbose) {
        this->print_flow();
    }

    return SUCCESS;
}

/*
 * CPLEX migration draft (review-only):
 * - This function is intentionally fully commented out so it does not affect current builds.
 * - It demonstrates how solve_flow() can be implemented with the CPLEX Callable Library network API.
 * - Warm start is shown via network basis get/copy calls.
 *
 * Notes:
 * 1) This uses CPXNET* routines (network simplex API), not Concert.
 * 2) Error handling is explicit at each API call for easier debugging during migration.
 * 3) Lower bounds are modeled directly on arcs, unlike the current OR-Tools transformation.
 */
/*
FAgentsPlanner::Result FAgentsPlanner::solve_flow_cplex_example() {
    // Reset solution containers, same behavior as current OR-Tools path.
    optimal_cost = 0;
    maximum_flow = 0;
    arc_flow.clear();
    flow.clear();

    // -----------------------------
    // 1) Build augmented min-cost flow model data
    // -----------------------------
    // We keep the same augmentation logic currently used:
    // - add total_source and total_sink
    // - connect commit/supply-demand helper arcs
    // - connect total_source -> source_idx and sink_idx -> total_sink
    const NodeIndex num_nodes = static_cast<NodeIndex>(idx_to_node.size());
    const ArcIndex num_arcs = static_cast<ArcIndex>(arc_tail.size());
    const ArcIndex augmented_num_arcs = num_arcs + 2 * static_cast<ArcIndex>(commit_edges_d_to_s.size()) + 2;
    const NodeIndex total_source = num_nodes;
    const NodeIndex total_sink = num_nodes + 1;
    const NodeIndex augmented_num_nodes = num_nodes + 2;

    // CPLEX expects contiguous arrays for the network.
    std::vector<int> fromnode;
    std::vector<int> tonode;
    std::vector<double> low;
    std::vector<double> up;
    std::vector<double> obj;
    std::vector<double> supply(augmented_num_nodes, 0.0);
    fromnode.reserve(augmented_num_arcs);
    tonode.reserve(augmented_num_arcs);
    low.reserve(augmented_num_arcs);
    up.reserve(augmented_num_arcs);
    obj.reserve(augmented_num_arcs);

    // Original planning arcs.
    for (ArcIndex a = 0; a < num_arcs; ++a) {
        fromnode.push_back(static_cast<int>(arc_tail[a]));
        tonode.push_back(static_cast<int>(arc_head[a]));
        low.push_back(0.0);                                 // Current model has implicit 0 lower bounds.
        up.push_back(static_cast<double>(arc_cap[a]));
        obj.push_back(static_cast<double>(arc_cost[a]));
    }

    // Commit helper arcs: total_source -> supply_node, demand_node -> total_sink.
    for (const auto &[demand_node, supply_node] : commit_edges_d_to_s) {
        fromnode.push_back(static_cast<int>(total_source));
        tonode.push_back(static_cast<int>(supply_node));
        low.push_back(0.0);
        up.push_back(static_cast<double>(CAP));
        obj.push_back(0.0);

        fromnode.push_back(static_cast<int>(demand_node));
        tonode.push_back(static_cast<int>(total_sink));
        low.push_back(0.0);
        up.push_back(static_cast<double>(CAP));
        obj.push_back(0.0);
    }

    // Global source/sink coupling arcs.
    fromnode.push_back(static_cast<int>(total_source));
    tonode.push_back(static_cast<int>(source_idx));
    low.push_back(0.0);
    up.push_back(static_cast<double>(num_of_agents));
    obj.push_back(0.0);

    fromnode.push_back(static_cast<int>(sink_idx));
    tonode.push_back(static_cast<int>(total_sink));
    low.push_back(0.0);
    up.push_back(static_cast<double>(num_of_agents));
    obj.push_back(0.0);

    // Net supplies: push all required flow from total_source to total_sink.
    const double required_flow = static_cast<double>(num_of_agents + commit_edges_d_to_s.size());
    supply[total_source] = required_flow;
    supply[total_sink] = -required_flow;

    // -----------------------------
    // 2) Create/refresh CPLEX network object
    // -----------------------------
    // Draft placeholders for persistent objects.
    // In production, keep env/net as class members so basis can persist across calls.
    CPXENVptr env = nullptr;
    CPXNETptr net = nullptr;
    int status = 0;

    env = CPXopenCPLEX(&status);
    if (env == nullptr || status) {
        throw std::runtime_error("CPLEX: CPXopenCPLEX failed.");
    }

    net = CPXNETcreateprob(env, &status, "wrp_flow");
    if (net == nullptr || status) {
        CPXcloseCPLEX(&env);
        throw std::runtime_error("CPLEX: CPXNETcreateprob failed.");
    }

    status = CPXNETcopynet(
        env,
        net,
        CPX_MIN,
        static_cast<int>(augmented_num_nodes),
        supply.data(),
        nullptr, // node names optional
        static_cast<int>(augmented_num_arcs),
        fromnode.data(),
        tonode.data(),
        low.data(),
        up.data(),
        obj.data(),
        nullptr  // arc names optional
    );
    if (status) {
        CPXNETfreeprob(env, &net);
        CPXcloseCPLEX(&env);
        throw std::runtime_error("CPLEX: CPXNETcopynet failed.");
    }

    // -----------------------------
    // 3) Optional warm start with saved basis
    // -----------------------------
    // If a previous basis exists and dimensions are compatible, copy it here.
    // Example design:
    // - Store previous arc/node basis arrays in class fields after each successful solve.
    // - Reinject with CPXNETcopybase before CPXNETprimopt.
    // status = CPXNETcopybase(env, net, saved_arc_stat.data(), saved_node_stat.data());
    // if (status) { ...fallback to cold start... }

    // -----------------------------
    // 4) Solve via network simplex
    // -----------------------------
    status = CPXNETprimopt(env, net);
    if (status) {
        CPXNETfreeprob(env, &net);
        CPXcloseCPLEX(&env);
        throw std::runtime_error("CPLEX: CPXNETprimopt failed.");
    }

    const int net_status = CPXNETgetstat(env, net);
    if (net_status != CPX_STAT_OPTIMAL) {
        // Map infeasible status to planner result.
        // Depending on CPLEX status codes used, adjust this check accordingly.
        CPXNETfreeprob(env, &net);
        CPXcloseCPLEX(&env);
        return INFEASIBLE;
    }

    // -----------------------------
    // 5) Read objective and arc flows
    // -----------------------------
    double objval = 0.0;
    status = CPXNETsolution(env, net, nullptr, &objval, nullptr, nullptr, nullptr, nullptr);
    if (status) {
        CPXNETfreeprob(env, &net);
        CPXcloseCPLEX(&env);
        throw std::runtime_error("CPLEX: CPXNETsolution failed.");
    }
    optimal_cost = static_cast<CostValue>(objval);

    std::vector<double> x(augmented_num_arcs, 0.0);
    status = CPXNETgetx(env, net, x.data(), 0, static_cast<int>(augmented_num_arcs) - 1);
    if (status) {
        CPXNETfreeprob(env, &net);
        CPXcloseCPLEX(&env);
        throw std::runtime_error("CPLEX: CPXNETgetx failed.");
    }

    // Keep only original arcs for downstream path extraction logic.
    arc_flow.resize(num_arcs, 0);
    for (ArcIndex a = 0; a < num_arcs; ++a) {
        arc_flow[a] = static_cast<FlowQuantity>(std::llround(x[a]));
        if (arc_flow[a] > 0) {
            flow[arc_tail[a]] = arc_head[a];
        }
    }

    // Required flow was enforced through supplies; if solved optimal, this is the delivered amount.
    maximum_flow = static_cast<FlowQuantity>(required_flow);

    // -----------------------------
    // 6) Save basis for next warm start (optional, but key for project goal)
    // -----------------------------
    // Example design:
    // std::vector<int> arc_stat(augmented_num_arcs), node_stat(augmented_num_nodes);
    // status = CPXNETgetbase(env, net, arc_stat.data(), node_stat.data());
    // if (!status) { saved_arc_stat = std::move(arc_stat); saved_node_stat = std::move(node_stat); }

    CPXNETfreeprob(env, &net);
    CPXcloseCPLEX(&env);
    return SUCCESS;
}
*/

/*
 * CPLEX migration draft (review-only) for cost updates:
 * Mirrors update_graph_edge_costs(), but applies updates directly on a persistent CPXNET model.
 */
/*
void FAgentsPlanner::update_graph_edge_costs_cplex_example(
    const vector<shared_ptr<TimedPath>> &obs_paths,
    CPXENVptr env,
    CPXNETptr net
) {
    // 1) Restore previously changed objective coefficients.
    //    In CPLEX, this is CPXNETchgobj over arc indices.
    if (!orig_edge_costs.empty()) {
        std::vector<int> arc_idx;
        std::vector<double> arc_obj;
        arc_idx.reserve(orig_edge_costs.size());
        arc_obj.reserve(orig_edge_costs.size());
        for (const auto &[a, old_cost] : orig_edge_costs) {
            arc_idx.push_back(static_cast<int>(a));
            arc_obj.push_back(static_cast<double>(old_cost));
            arc_cost[a] = old_cost;
        }
        int status = CPXNETchgobj(env, net, static_cast<int>(arc_idx.size()), arc_idx.data(), arc_obj.data());
        if (status) {
            throw std::runtime_error("CPLEX: CPXNETchgobj failed while restoring costs.");
        }
    }
    orig_edge_costs.clear();

    // 2) Reapply obstacle-driven edge penalties/bonuses.
    std::vector<int> changed_arcs;
    std::vector<double> changed_costs;
    for (auto &path : obs_paths) {
        int last_t = std::min(current_makespan, path->back().time);
        for (int t = path->front().time; t < last_t; t++) {
            Location loc1 = path->get_location_at_time(t);
            Location loc2 = path->get_location_at_time(t + 1);
            if (loc1 == loc2) {
                continue;
            }
            std::pair edge = (loc1 < loc2) ? std::make_pair(loc1, loc2) : std::make_pair(loc2, loc1);

            NodeIndex edge_in = node_to_idx.at(FlowNode(FlowNode::Type::EDGE_IN, t + 1, edge.first, edge.second));
            NodeIndex edge_out = node_to_idx.at(FlowNode(FlowNode::Type::EDGE_OUT, t + 1, edge.first, edge.second));
            ArcIndex arc = get_arc_idx(edge_in, edge_out);
            orig_edge_costs.emplace(arc, arc_cost[arc]);
            arc_cost[arc] = NO_COST;
            changed_arcs.push_back(static_cast<int>(arc));
            changed_costs.push_back(static_cast<double>(NO_COST));

            auto v2_out_it = node_to_idx.find(FlowNode(FlowNode::Type::OUT, t, loc2));
            if (v2_out_it == node_to_idx.end()) {
                continue;
            }
            NodeIndex v1_in = node_to_idx.at(FlowNode(FlowNode::Type::IN, t + 1, loc1));

            arc = get_arc_idx(v2_out_it->second, edge_in);
            orig_edge_costs.emplace(arc, arc_cost[arc]);
            arc_cost[arc] = UNIT_COST;
            changed_arcs.push_back(static_cast<int>(arc));
            changed_costs.push_back(static_cast<double>(UNIT_COST));

            arc = get_arc_idx(edge_out, v1_in);
            orig_edge_costs.emplace(arc, arc_cost[arc]);
            arc_cost[arc] = UNIT_COST;
            changed_arcs.push_back(static_cast<int>(arc));
            changed_costs.push_back(static_cast<double>(UNIT_COST));
        }
    }

    if (!changed_arcs.empty()) {
        int status = CPXNETchgobj(
            env,
            net,
            static_cast<int>(changed_arcs.size()),
            changed_arcs.data(),
            changed_costs.data()
        );
        if (status) {
            throw std::runtime_error("CPLEX: CPXNETchgobj failed while applying obstacle costs.");
        }
    }
}
*/

/*
 * CPLEX migration draft (review-only) for source/sink/commit updates:
 * Mirrors update_sources_and_sinks(), but applies capacity changes with CPXNETchgbds.
 */
/*
void FAgentsPlanner::update_sources_and_sinks_cplex_example(
    const vector<shared_ptr<Constraints>> &constraints_table,
    CPXENVptr env,
    CPXNETptr net
) {
    // 1) Restore previously deleted/disabled arcs to default capacity.
    if (!deleted_arcs.empty()) {
        std::vector<int> arc_idx;
        std::vector<char> lu;
        std::vector<double> bd;
        arc_idx.reserve(deleted_arcs.size());
        lu.reserve(deleted_arcs.size());
        bd.reserve(deleted_arcs.size());
        for (ArcIndex a : deleted_arcs) {
            arc_cap[a] = CAP;
            arc_idx.push_back(static_cast<int>(a));
            lu.push_back('U'); // change upper bound
            bd.push_back(static_cast<double>(CAP));
        }
        int status = CPXNETchgbds(env, net, static_cast<int>(arc_idx.size()), arc_idx.data(), lu.data(), bd.data());
        if (status) {
            throw std::runtime_error("CPLEX: CPXNETchgbds failed while restoring capacities.");
        }
    }

    commit_edges_d_to_s.clear();
    deleted_arcs.clear();

    // 2) Apply new positive-edge constraints by disabling specific EDGE_IN -> EDGE_OUT arcs.
    std::vector<int> arc_idx;
    std::vector<char> lu;
    std::vector<double> bd;
    for (auto &constraints : constraints_table) {
        for (auto &[t, edge] : constraints->get_positive_edge_constraints()) {
            if (t >= current_makespan) continue;
            NodeIndex v_from_out = node_to_idx.at(FlowNode(FlowNode::Type::OUT, t, edge.first));
            NodeIndex v_to_in = node_to_idx.at(FlowNode(FlowNode::Type::IN, t + 1, edge.second));
            commit_edges_d_to_s.emplace(v_from_out, v_to_in);

            std::pair edge_pair = (edge.first < edge.second) ? edge : std::make_pair(edge.second, edge.first);
            ArcIndex a = get_arc_idx(
                node_to_idx.at(FlowNode(FlowNode::Type::EDGE_IN, t + 1, edge_pair.first, edge_pair.second)),
                node_to_idx.at(FlowNode(FlowNode::Type::EDGE_OUT, t + 1, edge_pair.first, edge_pair.second))
            );
            arc_cap[a] = 0;
            deleted_arcs.emplace(a);
            arc_idx.push_back(static_cast<int>(a));
            lu.push_back('U');
            bd.push_back(0.0);
        }
    }

    if (!arc_idx.empty()) {
        int status = CPXNETchgbds(env, net, static_cast<int>(arc_idx.size()), arc_idx.data(), lu.data(), bd.data());
        if (status) {
            throw std::runtime_error("CPLEX: CPXNETchgbds failed while applying constraints.");
        }
    }
}
*/

/*
 * CPLEX migration draft (review-only) for graph growth:
 * This is the CPXNET-style replacement concept for OR-Tools Graph::Build + arc permutation.
 */
/*
void FAgentsPlanner::update_graph_cplex_example_incremental(int makespan, CPXENVptr env, CPXNETptr net) {
    // Keep all existing logic that computes which nodes/arcs should exist.
    // The difference is how we push the delta into the solver.
    //
    // Strategy:
    // 1) Detect new nodes and call CPXNETaddnodes(...).
    // 2) Detect new arcs and call CPXNETaddarcs(...).
    // 3) Avoid hard deletes when possible; set UB=0 for "removed" arcs (better for warm starts).
    // 4) Because CPXNET uses stable arc indices, no PermutedArc(...) function is needed.
    //
    // Pseudocode sketch:
    //   std::vector<double> new_supply(num_new_nodes, 0.0);
    //   CPXNETaddnodes(env, net, num_new_nodes, new_supply.data(), nullptr);
    //
    //   CPXNETaddarcs(env, net, num_new_arcs, from.data(), to.data(), low.data(), up.data(), cost.data(), nullptr);
    //
    //   // For sink arcs that are no longer active:
    //   CPXNETchgbds(env, net, n, arc_idx.data(), lu_upper.data(), new_ub.data()); // set UB=0
    //
    // After model modification, call CPXNETprimopt in solve.
    // Modified model invalidates current solution (expected), but basis warm start can still be injected next solve.
}
*/

// ORTOOLS_MIGRATION: Uses OR-Tools arc permutation produced by Graph::Build.
FAgentsPlanner::ArcIndex FAgentsPlanner::PermutedArc(ArcIndex arc) const {
    return arc < static_cast<int>(arc_permutation.size()) ? arc_permutation[arc] : arc;
}

FAgentsPlanner::Result
FAgentsPlanner::extract_agent_paths_and_detect_conflict(const vector<shared_ptr<TimedPath>> &obs_paths) {
    for (int a_i = 0; a_i < num_of_agents; ++a_i) {
        agent_paths[a_i] = std::make_shared<TimedPath>();
        agent_paths[a_i]->push_back(State(0, agent_start_locations[a_i]));
    }

    unordered_map<Location, Location> agent_edges_in_ts;
    realization_conflict = nullptr;

    vector<int> obs_conflicts_count(obs_paths.size(), 0);
    vector<int> obs_moves_counter(obs_paths.size(), 0);
    vector<shared_ptr<RealizationConflict>> obs_conflicts(obs_paths.size(), nullptr);
    for (int t = 1; t <= current_makespan; ++t) {
        agent_edges_in_ts.clear();
        for (int a_i = 0; a_i < num_of_agents; ++a_i) {
            Location last_loc = agent_paths[a_i]->back().location;
            NodeIndex v = node_to_idx.at(FlowNode(FlowNode::Type::OUT, t - 1, last_loc));
            auto it = commit_edges_d_to_s.find(v);
            NodeIndex next_v = (it != commit_edges_d_to_s.end()) ?
                                it->second :
                                flow.at(v);
            FlowNode next_node = idx_to_node.at(next_v);
            if (next_node.type == FlowNode::Type::IN) {
                agent_paths[a_i]->push_back(State(t, next_node.location1));
                agent_edges_in_ts.emplace(last_loc, next_node.location1);
            } else if (next_node.type == FlowNode::Type::EDGE_IN) {
                next_v = flow.at(flow.at(next_v));
                next_node = idx_to_node.at(next_v);
                assert(next_node.type == FlowNode::Type::IN);
                agent_paths[a_i]->push_back(State(t, next_node.location1));
                agent_edges_in_ts.emplace(last_loc, next_node.location1);
            } else {
                throw std::runtime_error("Invalid next node type.");
            }
        }

        for (int o_j = num_of_tasks - 1; o_j >= 0; o_j--) {
            Location last_loc = obs_paths[o_j]->get_location_at_time(t - 1);
            Location curr_loc = obs_paths[o_j]->get_location_at_time(t);
            if (last_loc == curr_loc) {
                continue;
            }
            obs_moves_counter[o_j]++;
            auto it = agent_edges_in_ts.find(last_loc);
            if (it != agent_edges_in_ts.end() && it->second == curr_loc) {
                continue;
            }
            realization_conflict = std::make_shared<RealizationConflict>(o_j, last_loc, curr_loc, t - 1);
            return CONFLICT;
        }
    }

    realization_conflict = nullptr;
    return SUCCESS;
}


void FAgentsPlanner::print_flow() {
    std::cout << "Minimum cost_type flow: " << optimal_cost << std::endl;
    for (int i = 0; i < num_of_agents; ++i) {
        std::cout << "---- Agent " << i << " ----" << std::endl;
        std::cout << "Cost     Flow" << std::endl;
        FlowNode curr_node = FlowNode(FlowNode::Type::OUT, 0, agent_start_locations[i]);
        NodeIndex curr_vertex = node_to_idx.at(curr_node);
        NodeIndex next_vertex, edge_out, v_in;
        while (curr_node.time != current_makespan) {
            int cost = 0;
            auto it = commit_edges_d_to_s.find(curr_vertex);
            if (it != commit_edges_d_to_s.end()) {
                next_vertex = it->second;
                FlowNode next_node = idx_to_node.at(next_vertex);
                std::cout << "[0] " << curr_node << " -> CE" << curr_node.location1 << "-" << next_node.location1
                          << " -> " << next_node << std::endl;
                curr_vertex = flow.at(next_vertex);
                curr_node = idx_to_node.at(curr_vertex);
                continue;
            }

            next_vertex = flow.at(curr_vertex);
            cost += arc_cost.at(get_arc_idx(curr_vertex, next_vertex));
            FlowNode next_node = idx_to_node.at(next_vertex);

            if (next_node.type == FlowNode::Type::IN)  {
                std::cout << "[" << cost << "]  " << curr_node << " -> " << next_node << std::endl;
                curr_vertex = flow.at(next_vertex);
                curr_node = idx_to_node.at(curr_vertex);
            } else {
                edge_out = flow.at(next_vertex);
                cost += arc_cost.at(get_arc_idx(next_vertex, edge_out));
                v_in = flow.at(edge_out);
                cost += arc_cost.at(get_arc_idx(edge_out, v_in));
                std::cout << "[" << cost << "]  " << curr_node << " -> E" << next_node.location1 << "-" << next_node.location2
                          << " -> " << idx_to_node.at(v_in) << std::endl;
                curr_vertex = flow.at(v_in);
                curr_node = idx_to_node.at(curr_vertex);
            }
        }
        std::cout << "[*]  " << curr_node << " -> SINK * DONE" << std::endl << std::endl;
    }
}
