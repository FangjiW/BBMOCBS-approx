#ifndef EXAMPLE_SHORTEST_PATH_HEURISTIC_H
#define EXAMPLE_SHORTEST_PATH_HEURISTIC_H

#include "Utils/Definitions.h"

// Precalculates heuristic based on Dijkstra shortest paths algorithm.
// On call to operator() returns the value of the heuristic in O(1)
class ShortestPathHeuristic {
private:
    size_t                  source;
    std::vector<NodePtr>    all_nodes;

    void compute(size_t cost_idx, const AdjacencyMatrix& adj_matrix);
public:
    ShortestPathHeuristic(size_t source, size_t graph_size, const AdjacencyMatrix &adj_matrix);
    std::vector<size_t> operator()(size_t node_id);
    void set_all_to_zero(){
        for (auto n: all_nodes){
            n->h = {0, 0};
        }
    }
};

#endif // EXAMPLE_SHORTEST_PATH_HEURISTIC_H
