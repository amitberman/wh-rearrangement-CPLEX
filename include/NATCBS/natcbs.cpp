#include "natcbs.hpp"


NATCBS::NATCBS(int map_rows, int map_cols, vector<vector<CellType>> &map, vector<Location> &agent_start_locations,
               vector<Location> &pickup_locations, vector<Location> &delivery_locations, bool verbose) :
        num_of_agents(static_cast<int>(agent_start_locations.size())),
        num_of_tasks(static_cast<int>(pickup_locations.size())),
        verbose(verbose),
        agents_planner(map_rows, map_cols, map, agent_start_locations, num_of_tasks, verbose) {
    for (int i = 0; i < num_of_tasks; i++) {
        int start_ts = abs(agent_start_locations[0].x - pickup_locations[i].x) +
                       abs(agent_start_locations[0].y - pickup_locations[i].y);
        for (int j = 1; j < num_of_agents; j++) {
            int dist = abs(agent_start_locations[j].x - pickup_locations[i].x) +
                       abs(agent_start_locations[j].y - pickup_locations[i].y);
            start_ts = std::min(start_ts, dist);
        }
        obs_start_ts.push_back(start_ts);
        State s_state(start_ts, pickup_locations[i]);
        solvers.emplace_back(map_rows, map_cols, map, s_state, delivery_locations[i]);
    }
}

NATCBS::NATCBS(Scenario &scenario, bool verbose) :
                NATCBS(scenario.map_rows, scenario.map_cols, scenario.map,
                       scenario.agent_start_locations, scenario.pickup_locations,scenario.delivery_locations,
                       verbose) {}


shared_ptr<Plan> NATCBS::solve(int timeout) {
    bool time_limit_exceeded;
    return solve(timeout, time_limit_exceeded);
}

shared_ptr<Plan> NATCBS::solve(int time_limit, bool& time_limit_exceeded) {
    time_limit_exceeded = false;
    Timer solve_timer;
    solve_timer.reset();
    OpenList open_list;
    open_list.push(get_root_node());
    iterations = 0;
    AgentsPlanner::Result ap_result;

    shared_ptr<NATCBSNode> node;
    bool expand = true;

    while (!open_list.empty() || !expand) {
        solve_timer.stop();
        if (solve_timer.elapsed_seconds() > time_limit) {
            time_limit_exceeded = true;
            return nullptr;
        }
        if (expand) {
            node = open_list.top();
            node->iteration = ++iterations;
            open_list.pop();
        } else {
            expand = true;
        }

//        node = open_list.top();
//        node->iteration = iterations++;
//        open_list.pop();
        if (verbose){
            std::cout << "Iteration: " << iterations << std::endl;
        }

        shared_ptr<Conflict> obs_conflict = node->get_first_obs_conflict();

        if (obs_conflict) {
            if (verbose) {
                std::cout << "Obs conflict detected: " << *obs_conflict << std::endl;
                std::cout << "[NATCBS] Split obstacle conflict" << std::endl;
            }
            auto children = resolve_obs_conflict(node, obs_conflict);
            if (children.first) {
                open_list.push(children.first);
            }
            if (children.second) {
                open_list.push(children.second);
            }
            continue;
        }

        if (verbose) {
            std::cout << "No obs conflict detected, planning agent paths..." << std::endl;
        }

        int makespan = node->cost;

        ap_result = agents_planner.find_paths(node->obs_paths, node->constraint_table, makespan);
        shared_ptr<RealizationConflict> realization_conflict = agents_planner.realization_conflict;

        if (ap_result == AgentsPlanner::INFEASIBLE) {
            // discard node, it is infeasible
            if (verbose) {
                std::cout << "Node is infeasible, skipping" << std::endl;
            }
            continue;
        } else if (ap_result == AgentsPlanner::SUCCESS) {
            if (verbose) {
                std::cout << "No realization conflict detected, returning plan" << std::endl;
            }
            return std::make_shared<Plan>(agents_planner.agent_paths, node->obs_paths);
        }

        assert(ap_result == AgentsPlanner::CONFLICT);
        if (verbose) {
            std::cout << "Realization conflict detected: " << *realization_conflict << std::endl;
            std::cout << "[NATCBS] Split realization conflict" << std::endl;
        }

        auto children = resolve_realization_conflict(node, realization_conflict);
        if (children.first) {
            open_list.push(children.first);
        }
        if (children.second) {
            open_list.push(children.second);
        }

    }
    return nullptr;
}

shared_ptr<NATCBSNode> NATCBS::get_root_node() {
    vector<shared_ptr<Constraints>> constraint_table(num_of_tasks, std::make_shared<Constraints>(true));
    vector<shared_ptr<TimedPath>> obs_paths(num_of_tasks, nullptr);
    int cost = 0;
    for (int i = 0; i < num_of_tasks; i++) {
        obs_paths[i] = plan_single_obs_path(i, constraint_table[i]);
        obs_paths[i]->erase_from_beginning();
        if (obs_paths[i]->size() <= 1) {
            continue;
        }
        cost = std::max(cost, obs_paths[i]->back().time);
    }
    return std::make_shared<NATCBSNode>(node_count++, obs_paths, cost, constraint_table);
}

shared_ptr<TimedPath> NATCBS::plan_single_obs_path(int obs_id, shared_ptr<Constraints> constraints) {
    shared_ptr<TimedPath> path = solvers[obs_id].find_path_use_landmarks(constraints);
    if (path != nullptr)
        path->erase_from_beginning();
    return path;
}

std::pair<shared_ptr<NATCBSNode>, shared_ptr<NATCBSNode>>
NATCBS::resolve_obs_conflict(const shared_ptr<NATCBSNode>& node, const shared_ptr<Conflict>& conflict) {
    VertexConstraint v_cons(conflict->time, conflict->loc1);
    EdgeConstraint e_cons(conflict->time, conflict->loc1, conflict->loc2);
    EdgeConstraint e_cons_reversed(conflict->time, conflict->loc2, conflict->loc1);

    // negative constraint
    shared_ptr<NATCBSNode> neg_child = nullptr;
    if (conflict->time >= obs_start_ts[conflict->agent1]) {
        shared_ptr<Constraints> neg_cons = std::make_shared<Constraints>(*node->constraint_table[conflict->agent1]);
        if (conflict->type == Conflict::Type::Vertex)
            neg_cons->add_negative_vertex_constraint(v_cons);
        else
            neg_cons->add_negative_edge_constraint(e_cons);
        shared_ptr<NATCBSNode> tmp_neg_child = std::make_shared<NATCBSNode>(node_count++, node);
        tmp_neg_child->constraint_table[conflict->agent1] = neg_cons;
        shared_ptr<TimedPath> neg_path = plan_single_obs_path(conflict->agent1, neg_cons);
        if (neg_path) {
            neg_child = tmp_neg_child;
            neg_child->cost = std::max(neg_child->cost, neg_path->back().time);
            neg_child->obs_paths[conflict->agent1] = neg_path;
        }
    }

    if (conflict->time < obs_start_ts[conflict->agent2]) {
        return {neg_child, nullptr};
    }

    // positive constraint
    shared_ptr<NATCBSNode> pos_child = std::make_shared<NATCBSNode>(node_count++, node);
    for (int i = 0; i < num_of_tasks; i++) {
        shared_ptr<Constraints> pos_cons = std::make_shared<Constraints>(*node->constraint_table[i]);
        pos_child->constraint_table[i] = pos_cons;

        if (conflict->type == Conflict::Type::Vertex){
            if (i == conflict->agent1) {
                pos_cons->add_positive_vertex_constraint(v_cons);
                continue;
            }
            pos_cons->add_negative_vertex_constraint(v_cons);
            if (i < conflict->agent2 || node->obs_paths.at(i)->get_location_at_time(conflict->time) != conflict->loc1) {
                continue;
            }
        } else {
            // edge conflict
            if (i == conflict->agent1) {
                pos_cons->add_positive_edge_constraint(e_cons);
                continue;
            }
            pos_cons->add_negative_edge_constraint(e_cons_reversed);
            if (i < conflict->agent2 || (node->obs_paths.at(i)->get_location_at_time(conflict->time) != conflict->loc2 &&
                     node->obs_paths.at(i)->get_location_at_time(conflict->time + 1) != conflict->loc1)) {
                continue;
            }
        }
        shared_ptr<TimedPath> pos_path = plan_single_obs_path(i, pos_cons);
        if (!pos_path){
            pos_child = nullptr;
            break;
        }
        pos_child->cost = std::max(pos_child->cost, pos_path->back().time);
        pos_child->obs_paths[i] = pos_path;
    }
    return {neg_child, pos_child};
}

std::pair<shared_ptr<NATCBSNode>, shared_ptr<NATCBSNode>>
NATCBS::resolve_realization_conflict(const shared_ptr<NATCBSNode> &node, const shared_ptr<RealizationConflict> &conflict) {
    EdgeConstraint e_cons(conflict->time, conflict->location1, conflict->location2);

    // negative constraint
    shared_ptr<NATCBSNode> neg_child = std::make_shared<NATCBSNode>(node_count++, node);
    shared_ptr<Constraints> neg_cons = std::make_shared<Constraints>(*node->constraint_table[conflict->obs_id]);
    neg_cons->add_negative_edge_constraint(e_cons);
    neg_child->constraint_table[conflict->obs_id] = neg_cons;
    shared_ptr<TimedPath> neg_path = plan_single_obs_path(conflict->obs_id, neg_cons);
    if (neg_path) {
        neg_child->cost = std::max(neg_child->cost, neg_path->back().time);
        neg_child->obs_paths[conflict->obs_id] = neg_path;
    } else {
        neg_child = nullptr;
    }

    // positive constraint
    EdgeConstraint e_cons_reversed(conflict->time, conflict->location2, conflict->location1);
    VertexConstraint v_cons1(conflict->time, conflict->location1);
    VertexConstraint v_cons2(conflict->time + 1, conflict->location2);
    shared_ptr<NATCBSNode> pos_child = std::make_shared<NATCBSNode>(node_count++, node);
    for (int i = 0; i < num_of_tasks; i++) {
        shared_ptr<Constraints> pos_cons = std::make_shared<Constraints>(*node->constraint_table[i]);
        pos_child->constraint_table[i] = pos_cons;

        if (i == conflict->obs_id) {
            pos_cons->add_positive_edge_constraint(e_cons);
        } else {
            pos_cons->add_negative_vertex_constraint(v_cons1);
            pos_cons->add_negative_vertex_constraint(v_cons2);
            pos_cons->add_negative_edge_constraint(e_cons_reversed);
        }
    }

    return {neg_child, pos_child};
}

