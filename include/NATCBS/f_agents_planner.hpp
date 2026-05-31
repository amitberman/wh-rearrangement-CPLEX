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

    // Optional topology mode for warm-start experiments.
    // When enabled, the time-expanded graph uses all passable map locations/edges
    // instead of pruning by dynamic reachability.
    bool full_topology_mode_enabled = false;
    vector<Location> full_topology_locations;
    vector<std::pair<Location, Location>> full_topology_edges;
    unordered_map<Location, int> full_topology_dist_from_starts;

    unordered_map<ArcIndex, CostValue> orig_edge_costs;
    unordered_map<NodeIndex, NodeIndex> commit_edges_d_to_s;
    unordered_set<ArcIndex> deleted_arcs;

#ifdef FLOW_BACKEND_ORTOOLS
    unordered_map<int, vector<ArcIndex>> makespan_to_arc_permutation;
#endif

#ifdef FLOW_BACKEND_CPLEX
    // ===== CPLEX Warm-Start Persistent State =====
    // These members persist across NATCBS iterations to enable warm-starting
    // of the network simplex solver with previously computed basis information.
    
    // CPLEX environment pointer: reused across all flow solves
    // Allocated on first use by ensure_cplex_model(), freed in destructor
    CPXENVptr cplex_env = nullptr;
    
    // CPLEX network model pointer: persistent min-cost flow network structure
    // Allocated on first use by ensure_cplex_model(), freed in destructor
    CPXNETptr cplex_net = nullptr;
    
    // Runtime toggle: controls whether to reuse previously computed simplex basis
    // Read from environment variable MAWR_CPLEX_WARM_START (default: 1)
    // When enabled: CPXNETcopybase() injects saved basis before solving
    // When disabled: solver starts from scratch (cold-start)
    bool cplex_warm_start_enabled = true;
    
    // Runtime toggle: controls whether to reuse and update the persistent model
    // Read from environment variable MAWR_CPLEX_REUSE_MODEL (default: 1)
    // When enabled: differential updates via CPXNETchgbds()/CPXNETchgobj()
    // When disabled: model is rebuilt via CPXNETcopynet() on every iteration
    bool cplex_reuse_model_enabled = true;
    
    // Saved network simplex basis from previous iteration
    // saved_arc_basis[a] = CPX status code for arc a (basic, lower bound, upper bound)
    // saved_node_basis[n] = CPX status code for node n (basic, free, or at bound)
    vector<int> saved_arc_basis;
    vector<int> saved_node_basis;
    
    // Dimensions of saved basis: validate compatibility before warm-start injection
    int saved_basis_arc_count = -1;
    int saved_basis_node_count = -1;
    
    // Topology fingerprint: detects when network structure changes.
    // Used for topology_unchanged check: if fromnode/tonode unchanged,
    // can apply differential updates; otherwise must rebuild model.
    vector<int> prev_fromnode;
    vector<int> prev_tonode;
    
    // Previous network dimensions: used to detect topology changes.
    int prev_node_count = -1;
    int prev_arc_count = -1;

    // Lazy initialization: ensures CPLEX environment and network model are created
    void ensure_cplex_model();
    
    // Reset basis state: called when topology changes or solve fails
    void clear_saved_basis();
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
    ~FAgentsPlanner();

    Result find_paths(const vector<shared_ptr<TimedPath>> &obs_paths,
                      const vector<shared_ptr<Constraints>> &constraints_table,
                      int makespan);

    Result extract_agent_paths_and_detect_conflict(const vector<shared_ptr<TimedPath>> &obs_paths);
};

#endif // MAWR_F_AGENTS_PLANNER_HPP