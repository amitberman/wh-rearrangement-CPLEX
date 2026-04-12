#include "f_agents_planner.hpp"

#include <cstdlib>
#include <queue>

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
    new_reached_locations.reserve(num_of_agents * 4);
    for (int i = 0; i < num_of_agents; ++i) {
        NodeIndex idx = get_node_idx(FlowNode(FlowNode::Type::OUT, 0, agent_start_locations[i]));
        add_arc(source_idx, idx, NO_COST);
        reached_locations.emplace_back(agent_start_locations[i]);
        new_reached_locations.insert(agent_start_locations[i]);
    }

    if (const char *full_topology_env = std::getenv("MAWR_FULL_TOPOLOGY_MODE")) {
        full_topology_mode_enabled = std::atoi(full_topology_env) != 0;
    }

    if (full_topology_mode_enabled) {
        full_topology_locations.reserve(map_rows * map_cols);
        full_topology_edges.reserve(map_rows * map_cols * 2 - map_rows - map_cols);

        for (int x = 0; x < map_rows; ++x) {
            for (int y = 0; y < map_cols; ++y) {
                if (map[x][y] == CellType::BLOCKED) {
                    continue;
                }
                full_topology_locations.emplace_back(x, y);
            }
        }

        for (int x = 0; x < map_rows; ++x) {
            for (int y = 0; y < map_cols; ++y) {
                if (map[x][y] == CellType::BLOCKED) {
                    continue;
                }
                const Location u(x, y);
                for (int d = 0; d < 4; ++d) {
                    const int nx = x + dx[d];
                    const int ny = y + dy[d];
                    if (nx < 0 || nx >= map_rows || ny < 0 || ny >= map_cols) {
                        continue;
                    }
                    if (map[nx][ny] == CellType::BLOCKED) {
                        continue;
                    }
                    const Location v(nx, ny);
                    if (u < v) {
                        full_topology_edges.emplace_back(u, v);
                    }
                }
            }
        }

        // Precompute shortest-time reachability from any agent start.
        // In full-topology mode, we keep all arcs but set UB=0 on arcs whose
        // tail-side move cannot be reached at the current timestep.
        const int INF_DIST = 1e9;
        full_topology_dist_from_starts.reserve(full_topology_locations.size());
        for (const auto &loc : full_topology_locations) {
            full_topology_dist_from_starts.emplace(loc, INF_DIST);
        }

        std::queue<Location> q;
        for (const auto &start_loc : agent_start_locations) {
            auto it = full_topology_dist_from_starts.find(start_loc);
            if (it != full_topology_dist_from_starts.end() && it->second > 0) {
                it->second = 0;
                q.push(start_loc);
            }
        }

        while (!q.empty()) {
            const Location u = q.front();
            q.pop();
            const int du = full_topology_dist_from_starts.at(u);
            for (int d = 0; d < 4; ++d) {
                const int nx = u.x + dx[d];
                const int ny = u.y + dy[d];
                if (nx < 0 || nx >= map_rows || ny < 0 || ny >= map_cols) {
                    continue;
                }
                if (map[nx][ny] == CellType::BLOCKED) {
                    continue;
                }
                const Location v(nx, ny);
                auto it = full_topology_dist_from_starts.find(v);
                if (it == full_topology_dist_from_starts.end()) {
                    continue;
                }
                if (it->second > du + 1) {
                    it->second = du + 1;
                    q.push(v);
                }
            }
        }
    }

#ifdef FLOW_BACKEND_CPLEX
    // ===== CPLEX Warm-Start Configuration (from environment variables) =====
    // MAWR_CPLEX_WARM_START: controls whether to reuse the simplex basis from previous iteration
    // - When set to 1 (default): CPXNETcopybase() loads saved basis before each solve
    // - When set to 0: solver ignores saved basis and performs cold-start
    // - Used in solve_flow() to determine if basis warm-start can be applied
    if (const char *warm_start_env = std::getenv("MAWR_CPLEX_WARM_START")) {
        cplex_warm_start_enabled = std::atoi(warm_start_env) != 0;
    }
    
    // MAWR_CPLEX_REUSE_MODEL: controls whether to reuse and incrementally update the model
    // - When set to 1 (default): persistent model is updated via CPXNETchgbds/chgobj (differential updates)
    // - When set to 0: model is discarded and rebuilt via CPXNETcopynet on every iteration
    // - Combined with MAWR_CPLEX_WARM_START=1, achieves maximum warm-start benefit
    // - Setting to 0 disables incremental updates (reverts to per-iteration rebuild)
    if (const char *reuse_env = std::getenv("MAWR_CPLEX_REUSE_MODEL")) {
        cplex_reuse_model_enabled = std::atoi(reuse_env) != 0;
    }
#endif
}

FAgentsPlanner::~FAgentsPlanner() {
#ifdef FLOW_BACKEND_CPLEX
    // ===== Destructor: Clean up persistent CPLEX resources =====
    // Must free network model before closing environment
    if (cplex_net != nullptr) {
        CPXNETfreeprob(cplex_env, &cplex_net);
    }
    // Then close the CPLEX environment
    if (cplex_env != nullptr) {
        CPXcloseCPLEX(&cplex_env);
    }
#endif
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
    for (const Location &loc : new_reached_locations) {
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
                                   [new_loc](const Location loc2) { return loc2 == new_loc; }));
                continue;
            }
            if (next_reached_locations.count(new_loc) || new_reached_locations.count(new_loc)) {
                assert(std::all_of(map_edges.begin(), map_edges.end(),
                                   [edge](const std::pair<Location, Location> &e) { return e != edge; }));
                assert(std::any_of(reached_locations.begin(), reached_locations.end(),
                                   [new_loc](const Location loc2) { return loc2 == new_loc; }));
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
                               [new_loc](const Location loc2) { return loc2 != new_loc; }));
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

    if (full_topology_mode_enabled) {
        auto can_reach_by_time = [&](const Location &loc, int t) -> bool {
            auto it = full_topology_dist_from_starts.find(loc);
            if (it == full_topology_dist_from_starts.end()) {
                return false;
            }
            return it->second <= t;
        };

        auto add_arc_with_cap = [&](NodeIndex tail, NodeIndex head, CostValue cost, bool active) {
            this->add_arc(tail, head, cost);
            arc_cap.back() = active ? CAP : 0;
        };

        for (int t = max_makespan + 1; t <= makespan; ++t) {
            // v_t-1_out -> v_t_in for all passable locations.
            for (const auto &loc : full_topology_locations) {
                NodeIndex v_out = get_node_idx(FlowNode(FlowNode::Type::OUT, t - 1, loc));
                NodeIndex v_in = get_node_idx(FlowNode(FlowNode::Type::IN, t, loc));
                add_arc_with_cap(v_out, v_in, UNIT_COST, can_reach_by_time(loc, t - 1));
            }

            // Edge gadgets for all passable undirected map edges.
            for (const auto &[loc1, loc2] : full_topology_edges) {
                NodeIndex v_1_out = get_node_idx(FlowNode(FlowNode::Type::OUT, t - 1, loc1));
                NodeIndex v_2_out = get_node_idx(FlowNode(FlowNode::Type::OUT, t - 1, loc2));

                NodeIndex edge_in = get_node_idx(FlowNode(FlowNode::Type::EDGE_IN, t, loc1, loc2));
                const bool loc1_active_prev = can_reach_by_time(loc1, t - 1);
                const bool loc2_active_prev = can_reach_by_time(loc2, t - 1);
                add_arc_with_cap(v_1_out, edge_in, NO_COST, loc1_active_prev);
                add_arc_with_cap(v_2_out, edge_in, NO_COST, loc2_active_prev);

                NodeIndex edge_out = get_node_idx(FlowNode(FlowNode::Type::EDGE_OUT, t, loc1, loc2));
                add_arc_with_cap(edge_in, edge_out, UNIT_COST, loc1_active_prev || loc2_active_prev);

                NodeIndex v_1_in = get_node_idx(FlowNode(FlowNode::Type::IN, t, loc1));
                NodeIndex v_2_in = get_node_idx(FlowNode(FlowNode::Type::IN, t, loc2));

                // edge_out -> v_1_in is used by a move that starts from loc2 at t-1.
                add_arc_with_cap(edge_out, v_1_in, NO_COST, loc2_active_prev);
                // edge_out -> v_2_in is used by a move that starts from loc1 at t-1.
                add_arc_with_cap(edge_out, v_2_in, NO_COST, loc1_active_prev);
            }

            // v_t_in -> v_t_out for all passable locations.
            for (const auto &loc : full_topology_locations) {
                NodeIndex v_in = node_to_idx.at(FlowNode(FlowNode::Type::IN, t, loc));
                NodeIndex v_out = get_node_idx(FlowNode(FlowNode::Type::OUT, t, loc));
                add_arc_with_cap(v_in, v_out, NO_COST, can_reach_by_time(loc, t));
            }
        }

        // v_T_out -> sink for all passable locations.
        for (const auto &loc : full_topology_locations) {
            NodeIndex v_out = get_node_idx(FlowNode(FlowNode::Type::OUT, makespan, loc));
            this->add_arc_to_sink(v_out);
            arc_cap.back() = can_reach_by_time(loc, makespan) ? CAP : 0;
        }

        current_makespan = makespan;
        max_makespan = std::max(max_makespan, makespan);
        return;
    }

    for (int t = max_makespan + 1; t <= makespan; t++) {
        // v_t-1_out -> v_t_in
        for (auto &loc : reached_locations) {
            NodeIndex v_out = node_to_idx.at(FlowNode(FlowNode::Type::OUT, t - 1, loc));
            NodeIndex v_in = get_node_idx(FlowNode(FlowNode::Type::IN, t, loc));
            this->add_arc(v_out, v_in, UNIT_COST);
        }

        // edge gadgets
        update_map_edges();
        for (const auto &[loc1, loc2] : map_edges) {
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
        FlowNode f_node(FlowNode(FlowNode::Type::OUT, makespan, loc));
        auto it = node_to_idx.find(f_node);
        if (it == node_to_idx.end()) continue;
        this->add_arc_to_sink(it->second);
    }

    current_makespan = makespan;
    max_makespan = std::max(max_makespan, makespan);
}

void FAgentsPlanner::update_graph_edge_costs(const vector<shared_ptr<TimedPath>> &obs_paths) {
    for (auto &[e, c] : orig_edge_costs) {
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
#ifdef FLOW_BACKEND_ORTOOLS
    for (auto &e : deleted_arcs) {
        arc_cap[e] = CAP;
    }
    deleted_arcs.clear();
#endif

    for (auto &constraints : constraints_table) {
        for (auto &[t, edge] : constraints->get_positive_edge_constraints()) {
            if (t >= current_makespan) continue;
            NodeIndex v_from_out = node_to_idx.at(FlowNode(FlowNode::Type::OUT, t, edge.first));
            NodeIndex v_to_in = node_to_idx.at(FlowNode(FlowNode::Type::IN, t + 1, edge.second));
            commit_edges_d_to_s.emplace(v_from_out, v_to_in);

#ifdef FLOW_BACKEND_ORTOOLS
            std::pair edge_pair = (edge.first < edge.second) ? edge : std::make_pair(edge.second, edge.first);
            ArcIndex arc = get_arc_idx(node_to_idx.at(FlowNode(FlowNode::Type::EDGE_IN, t + 1, edge_pair.first, edge_pair.second)),
                                       node_to_idx.at(FlowNode(FlowNode::Type::EDGE_OUT, t + 1, edge_pair.first, edge_pair.second)));
            arc_cap[arc] = 0;
            deleted_arcs.emplace(arc);
#endif
        }
    }
}

FAgentsPlanner::Result FAgentsPlanner::find_paths(const vector<shared_ptr<TimedPath>> &obs_paths,
                                            const vector<shared_ptr<Constraints>> &constraints_table, int makespan) {
    update_graph(makespan);
    update_graph_edge_costs(obs_paths);
    update_sources_and_sinks(constraints_table);

    const auto result = this->solve_flow();
    if (result != SUCCESS) {
        return result;
    }

    return this->extract_agent_paths_and_detect_conflict(obs_paths);
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