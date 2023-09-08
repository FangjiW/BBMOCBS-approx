#include <limits>
#include <memory>
#include <algorithm>

#include "ShortestPathHeuristic.h"
#include <AbstractSolver.h>

extern std::unordered_map<size_t, std::vector<int>> id2coord;

ShortestPathHeuristic::ShortestPathHeuristic(size_t source, size_t graph_size, const AdjacencyMatrix &adj_matrix, VertexConstraint& vertex_constraints, EdgeConstraint& edge_constraints, int turn_mode, int turn_cost)
    : source(source), all_nodes(graph_size+1) {
    size_t num_of_objectives = adj_matrix.get_num_of_objectives();
    size_t i = 0;
//     for (auto node_iter = this->all_nodes.begin(); node_iter != this->all_nodes.end(); node_iter++) {
//         *node_iter = std::make_shared<Node>(i++, std::vector<size_t>(num_of_objectives, 0), std::vector<size_t>(num_of_objectives, MAX_COST));
//  }

    for (int j=0; j < num_of_objectives; j ++){
        bool if_turn = j == turn_mode ? true : false;
        compute(j, adj_matrix, vertex_constraints, edge_constraints, if_turn, turn_cost);
    }
}

std::vector<size_t> ShortestPathHeuristic::operator()(size_t node_id, size_t parent_id, bool if_turn) {
    if(if_turn){
        if(parent_id == -1){
            return all_nodes.at(node_id).front()->h;
        }
        else{
            for(auto ele: all_nodes.at(node_id)){
                if(ele->parent->id == parent_id){
                    return ele->h;
                }
            }
            std::cout << "(ShortestPathHeuristic::operator()) ERROR";
        }
    }else{
        return this->all_nodes.at(node_id).front()->h;
    }
}


// Implements Dijkstra shortest path algorithm per cost_idx cost function
void ShortestPathHeuristic::compute(size_t cost_idx, const AdjacencyMatrix &adj_matrix, VertexConstraint& vertex_constraints, EdgeConstraint& edge_constraints, bool if_turn, int turn_cost) {
    // Init all heuristics to MAX_COST
    // for (auto node_iter = this->all_nodes.begin(); node_iter != this->all_nodes.end(); node_iter++) {
    //     (*node_iter)->h[cost_idx] = MAX_COST;
    // }
    // std::cout << "no";
    // getchar();
    size_t num_of_objectives = adj_matrix.get_num_of_objectives();
    NodePtr node;
    NodePtr next;

    // Init open heap
    Node::more_than_specific_heurisitic_cost more_than(cost_idx);
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    auto root_node = std::make_shared<Node>(this->source, std::vector<size_t>(num_of_objectives, 0), std::vector<size_t>(num_of_objectives, 0));
    open.push_back(root_node);
    std::vector<std::vector<std::pair<size_t, size_t>>>  parent_h_combo(all_nodes.size());      // in inverse graph
    parent_h_combo.at(this->source).push_back(std::make_pair(-1, 0));
    
    // std::cout << "ifturn:" << if_turn << std::endl;
    // getchar();
    if(!if_turn){
        if(all_nodes.at(this->source).empty()){
            all_nodes.at(this->source).push_back(root_node);
        }else{
            for(auto& ele: all_nodes.at(this->source)){
                ele->h.at(cost_idx) = 0;
            }
        }
    }
    while (open.empty() == false) {
        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        if(if_turn && outgoing_edges.empty()){
            if(all_nodes.at(node->id).empty()){
                all_nodes.at(node->id).push_back(std::make_shared<Node>(node->id, node->g, node->h));
            }else{
                if(all_nodes.at(node->id).front()->h.at(cost_idx) <= node->h.at(cost_idx)){
                    continue;
                }else{
                    all_nodes.at(node->id).front()->h.at(cost_idx) = node->h.at(cost_idx);
                }
            }
            continue;
        }
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            NodePtr _next = std::make_shared<Node>(p_edge->target, std::vector<size_t>(num_of_objectives, 0), std::vector<size_t>(num_of_objectives, MAX_COST));

            // Constraint check
            // if(is_constraint(next, vertex_constraints, edge_constraints)){
            //     continue;
            // }

            size_t  next_h = node->h[cost_idx]+p_edge->cost[cost_idx];
            if(if_turn){
                if(node->parent == nullptr){
                    if(_next->id != node->id){
                        next_h += turn_cost;
                    }
                }else{
                    int x0 = id2coord[node->parent->id].at(0);
                    int y0 = id2coord[node->parent->id].at(1);
                    int x1 = id2coord[node->id].at(0);
                    int y1 = id2coord[node->id].at(1);
                    int x2 = id2coord[_next->id].at(0);
                    int y2 = id2coord[_next->id].at(1);
                    if(x1-x0 != x2-x1 || y1-y0 != y2-y1){
                        next_h += turn_cost;
                    }
                }
                // combo dominance check
                bool if_dominated = false;
                bool exist_same_parent_node = false;
                for(auto& ele: parent_h_combo.at(_next->id)){
                    if(node->id == ele.first){
                        exist_same_parent_node = true;
                        if(ele.second <= next_h){
                            if_dominated = true;
                            break;
                        }else{
                            ele.second = next_h;
                        }
                    }
                }
                if(if_dominated){
                    continue;
                }
                if(!exist_same_parent_node){
                    parent_h_combo.at(_next->id).push_back(std::make_pair(node->id, next_h));
                }
                bool exist_same_next_node = false;
                for(auto& ele: all_nodes.at(node->id)){
                    if(ele->parent->id == _next->id){
                        exist_same_next_node = true;
                        ele->h.at(cost_idx) = next_h - p_edge->cost[cost_idx];
                    }
                }
                if(!exist_same_next_node){
                    auto new_h = std::vector<size_t>(num_of_objectives, MAX_COST);
                    new_h.at(cost_idx) = next_h - p_edge->cost[cost_idx];
                    auto next = std::make_shared<Node>(*_next);
                    next->h.at(cost_idx) = next_h;
                    all_nodes.at(node->id).push_back(std::make_shared<Node>(node->id, node->g, new_h, next));
                }
            }else{
                _next->h.at(cost_idx) = next_h;
                if(all_nodes.at(_next->id).empty()){
                    all_nodes.at(_next->id).push_back(std::make_shared<Node>(_next->id, _next->g, _next->h, nullptr));
                }else{
                    // if(id2coord[node->id].at(0) == 10 && id2coord[node->id].at(1) == 8){
                    //     std::cout << "1   ";
                    //     std::cout << id2coord[_next->id].at(0) << "," << id2coord[_next->id].at(1) << "    " << next_h;
                    //     getchar();
                    // std::cout << all_nodes.at(_next->id).front()->h.at(cost_idx) << "  " << next_h;
                    // getchar();
                    // }
                    // if(id2coord[_next->id].at(0) == 9 && id2coord[_next->id].at(1) == 8){
                    //     std::cout << "2   ";
                    //     std::cout << id2coord[node->id].at(0) << "," << id2coord[node->id].at(1) << "       " << next_h << std::endl;
                    //     std::cout << all_nodes.at(_next->id).front()->h.at(cost_idx);
                    //     getchar();
                    // }
                    if(all_nodes.at(_next->id).front()->h.at(cost_idx) <= next_h){
                        continue;
                    }else{
                        for(auto& ele:all_nodes.at(_next->id)){
                            ele->h.at(cost_idx) = next_h;
                        }
                        // all_nodes.at(_next->id).front()->h.at(cost_idx) = next_h;
                    }
                }
            }
            // If not dominated push to queue
            _next->h.at(cost_idx) = next_h;
            _next->parent = node;
            open.push_back(_next);
            std::push_heap(open.begin(), open.end(), more_than);
        }
    }
    // std::cout << "ou";getchar();
    // if(if_turn){
    //     for(auto ele: all_nodes){
    //         if(id2coord[ele->id].size() == 0){
    //             break;
    //         }
    //         std::cout << id2coord[ele->id].at(0) << "," << id2coord[ele->id].at(1) << ":";
    //         std::cout << ele->h.at(cost_idx) << "   ";
    //     }
    //     std::cout << std::endl << std::endl;
    //     getchar();
    // }
    // for(auto ele: all_nodes){
    //     for(auto ele1: ele){
    //         std::cout << "id: " << id2coord[ele1->id].at(0) << "," << id2coord[ele1->id].at(1) << "   h:" << ele1->h.at(cost_idx) << ", " << std::endl;
    //     }
    // }
    // std::cout << "yes";
    // getchar();
}