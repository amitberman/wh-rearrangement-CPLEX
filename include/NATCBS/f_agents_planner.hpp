#ifndef MAWR_F_AGENTS_PLANNER_HPP
#define MAWR_F_AGENTS_PLANNER_HPP

#include "flow_node.hpp"
#include "../constraints/constraints.hpp"
#include "realization_conflict.hpp"

#ifdef FLOW_BACKEND_ORTOOLS
#include "ortools/graph/min_cost_flow.h"
#include "ortools/graph/graph.h"
#include <cstdint>
#endif

#ifdef FLOW_BACKEND_CPLEX
#include "ilcplex/cplex.h"
#include <cstdint>
#endif

class FAgentsPlanner {
public:
    enum Result {
        SUCCESS,
        CONFLICT,
        INFEASIBLE
    };

private:
    int map_rows, map_cols;
    vector<vector<CellType>> map;
    int num_of_agents;
    vector<Location> agent_start_locations;
    int num_of_tasks;
    bool verbose;

#ifdef FLOW_BACKEND_ORTOOLS
    typedef util::ReverseArcStaticGraph<> Graph;
    typedef Graph::NodeIndex NodeIndex;
    typedef Graph::ArcIndex ArcIndex;
    typedef operations_research::GenericMinCostFlow<Graph> MinCostFlow;
#elif defined(FLOW_BACKEND_CPLEX)
    typedef int32_t NodeIndex;
    typedef int32_t ArcIndex;
#else
#error "No flow backend selected. Define FLOW_BACKEND_ORTOOLS or FLOW_BACKEND_CPLEX."
#endif

    typedef int32_t FlowQuantity;
    typedef int32_t CostValue;

    constexpr static const int dx[4] = {0, 1, 0, -1};
    constexpr static const int dy[4] = {1, 0, -1, 0};
    constexpr static const FlowQuantity CAP = 1;
    constexpr static const CostValue NO_COST = 0;
    constexpr static const CostValue UNIT_COST = 1;

    vector<FlowNode> idx_to_node;
    unordered_map<FlowNode, NodeIndex> node_to_idx;
    FlowNode source_node;
    NodeIndex source_idx;
    FlowNode sink_node;
    NodeIndex sink_idx;
    int current_makespan;
    int max_makespan;
    int num_edges_before_sink;
    unordered_map<NodeIndex, unordered_map<NodeIndex, ArcIndex>> arc_map;

    vector<NodeIndex> arc_tail;
    vector<NodeIndex> arc_head;
    vector<CostValue> arc_cost;
    vector<FlowQuantity> arc_cap;

#ifdef FLOW_BACKEND_ORTOOLS
    vector<ArcIndex> arc_permutation;
#endif

    vector<FlowQuantity> arc_flow;
    unordered_map<NodeIndex, NodeIndex> flow;
    CostValue optimal_cost;
    FlowQuantity maximum_flow;

    vector<Location> reached_locations;
    unordered_set<Location> new_reached_locations;
    unordered_set<Location> recently_reached_locations;
    vector<std::pair<Location, Location>> map_edges;
    bool reached_all = false;

    unordered_map<ArcIndex, CostValue> orig_edge_costs;
    unordered_map<NodeIndex, NodeIndex> commit_edges_d_to_s;
    unordered_set<ArcIndex> deleted_arcs;

#ifdef FLOW_BACKEND_ORTOOLS
    unordered_map<int, vector<ArcIndex>> makespan_to_arc_permutation;
#endif

    unordered_map<int, array<int, 3>> makespan_graph_params;

    NodeIndex get_node_idx(const FlowNode &node);
    ArcIndex get_arc_idx(NodeIndex tail, NodeIndex head);
    void add_arc(NodeIndex tail, NodeIndex head, CostValue cost);
    void add_arc_to_sink(NodeIndex tail);
    void update_map_edges();
    void update_graph(int makespan);

    void update_graph_edge_costs(const vector<shared_ptr<TimedPath>> &obs_paths);
    void update_single_edge_cost(NodeIndex n1, NodeIndex n2, CostValue cost);
    void update_sources_and_sinks(const vector<shared_ptr<Constraints>> &constraints_table);

    Result solve_flow();

#ifdef FLOW_BACKEND_ORTOOLS
    ArcIndex PermutedArc(ArcIndex arc) const;
#endif

    void print_flow();

public:
    vector<shared_ptr<TimedPath>> agent_paths;
    shared_ptr<RealizationConflict> realization_conflict;

    FAgentsPlanner(int map_rows, int map_cols, const vector<vector<CellType>> &map,
                   vector<Location> &agent_start_locations,
                   int num_of_tasks, bool verbose);

    Result find_paths(const vector<shared_ptr<TimedPath>> &obs_paths,
                      const vector<shared_ptr<Constraints>> &constraints_table,
                      int makespan);

    Result extract_agent_paths_and_detect_conflict(const vector<shared_ptr<TimedPath>> &obs_paths);
};

#endif // MAWR_F_AGENTS_PLANNER_HPP