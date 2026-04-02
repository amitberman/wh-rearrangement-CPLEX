#include "f_agents_planner.hpp"
#include "ortools/graph/max_flow.h"

// ORTOOLS_MIGRATION: Direct OR-Tools dependency (GenericMaxFlow + GenericMinCostFlow).
FAgentsPlanner::Result FAgentsPlanner::solve_flow() {
    optimal_cost = 0;
    maximum_flow = 0;
    arc_flow.clear();
    flow.clear();

    const auto num_nodes = static_cast<NodeIndex>(idx_to_node.size());
    const auto num_arcs = static_cast<ArcIndex>(arc_tail.size());
    assert(num_nodes > 0);

    // Feasibility checking, and possible supply/demand adjustment, is done by:
    // 1. Creating a new source and sink node.
    // 2. Taking all nodes that have a non-zero supply or demand and connecting them to the source or sink
    //    respectively. The arc thus added has a capacity of the supply or demand.
    // 3. Computing the max flow between the new source and sink.
    // 4. Checking that the max flow is equal to the total supply/demand (and returning INFEASIBLE if it isn't).
    // 5. Running min-cost max-flow on this augmented graph, using the max flow computed in step 3 as the supply of
    //    the source and demand of the sink.
    const ArcIndex augmented_num_arcs =
        num_arcs + 2 * static_cast<ArcIndex>(commit_edges_d_to_s.size()) + 2;
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

    for (; arc < augmented_num_arcs - 2; ++arc) {
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
                if (verbose) {
                    std::cout << "f "
                              << idx_to_node.at(arc_tail.at(arc))
                              << " -> "
                              << idx_to_node.at(arc_head.at(arc))
                              << " "
                              << arc_flow[arc]
                              << std::endl;
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

// ORTOOLS_MIGRATION: Uses OR-Tools arc permutation produced by Graph::Build.
FAgentsPlanner::ArcIndex FAgentsPlanner::PermutedArc(ArcIndex arc) const {
    return arc < static_cast<int>(arc_permutation.size()) ? arc_permutation[arc] : arc;
}