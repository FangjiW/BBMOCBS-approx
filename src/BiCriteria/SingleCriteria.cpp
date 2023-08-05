




// #include "SingleCriteria.h"
// #include <unordered_set>

// AStar::AStar(const AdjacencyMatrix &adj_matrix):
//     adj_matrix(adj_matrix) {};


// NodePtr AStar::operator()(size_t source, size_t target, Heuristic &heuristic, Node::LEX_ORDER order) {

//     // Saving all the unused NodePtrs in a vector improves performace for some reason
//     std::unordered_set<size_t> closed;

//     // Init open heap
//     Node::more_than_lex more_than(order);
//     std::vector<NodePtr> open;
//     std::make_heap(open.begin(), open.end(), more_than);

//     auto node = std::make_shared<Node>(source, std::vector<size_t>({0,0}), heuristic(source));
//     open.push_back(node);
//     std::push_heap(open.begin(), open.end(), more_than);

//     while (open.empty() == false) {
//         // Pop min from queue and process
//         std::pop_heap(open.begin(), open.end(), more_than);
//         node = open.back();
//         open.pop_back();

//         if (closed.find(node->id) != closed.end()){
//             continue;
//         }
//         closed.insert(node->id);

//         // Dominance check
//         if (node->id == target) {
//             return node;
//         }

//         // Check to which neighbors we should extend the paths
//         const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
//         for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
//             size_t next_id = p_edge->target;
//             std::vector<size_t> next_g = {node->g[0]+p_edge->cost[0], node->g[1]+p_edge->cost[1]};
//             std::vector<size_t> next_h = heuristic(next_id);

//             if (closed.find(next_id) != closed.end()){
//                 continue;
//             }

//             // If not dominated create node and push to queue
//             // Creation is defered after dominance check as it is
//             // relatively computational heavy and should be avoided if possible
//             auto next = std::make_shared<Node>(next_id, next_g, next_h, node);

//             open.push_back(next);
//             std::push_heap(open.begin(), open.end(), more_than);

//             // closed.push_back(node);
//         }
//     }

// }
