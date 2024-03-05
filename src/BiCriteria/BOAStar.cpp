#include <memory>
#include <algorithm>
#include <time.h>
#include<stack>

#include "BOAStar.h"

BOAStar::BOAStar(const AdjacencyMatrix &adj_matrix, Pair<double> eps, int turn_mode, int turn_cost, const LoggerPtr logger) :
    AbstractSolver(adj_matrix, {eps[0], eps[1]}, turn_mode, turn_cost, logger) {}


void BOAStar::operator()(PathSet& solution_ids, CostSet& solution_apex_costs, CostSet& solution_real_costs, 
        size_t source, size_t target, Heuristic &heuristic, VertexConstraint& vertex_constraints, EdgeConstraint& edge_constraints,
        unsigned int time_limit, VertexCAT& vertex_cat, EdgeCAT& edge_cat, std::unordered_map<int, int>& conflict_num_map)  {
    // int time_limit = 300;
    // std::vector<NodePtr> solutions;
    SolutionSet solutions;
    start_time = std::clock();
    this->start_logging(source, target);

    NodePtr node;
    NodePtr next;

    // Saving all the unused NodePtrs in a vector improves performace for some reason
    std::vector<NodePtr> closed;

    // Vector to hold mininum cost of 2nd criteria per node
    std::vector<std::unordered_map<size_t, std::vector<std::pair<size_t, size_t>>>> min_g2(this->adj_matrix.size());    // unordered_map<time, vector<pair<parent_id, cost>>>

    size_t  min_g2_target = UINT64_MAX;

    // Init open heap
    Node::more_than_full_cost more_than;
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    bool if_turn = turn_mode == -1 ? false : true;
    node = std::make_shared<Node>(source, std::vector<size_t>(2,0), heuristic(source, -1, if_turn));
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);
            extern std::unordered_map<size_t, std::vector<int>> id2coord;
            
    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){

            this->end_logging(solutions, false);
            break;
        }

        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();
        num_generation +=1;

        // Dominance check
        if(node->id == target){
            if((1+this->eps_prune[1])*node->f[1] >= min_g2_target){
                continue;
            }else{
                min_g2_target = node->g[1];
            }
        }else{
            bool if_dominated = false;
            bool exist_same_parent_pair = false;
            if(((1+this->eps_prune[1])*node->f[1]) >= min_g2_target){
                if_dominated = true;
            }else if(min_g2[node->id].count(node->t)){
                for(auto& ele: min_g2[node->id][node->t]){
                    if(if_turn && !(ele.first == -1 && node->parent == nullptr)){
                        if(ele.first != node->parent->id){
                            continue;
                        }
                    }
                    exist_same_parent_pair = true;
                    if(node->g[1] >= ele.second){
                        if_dominated = true;
                        break;
                    }else{
                        ele.second = node->g[1];
                    }
                }
            }
            if (if_dominated) {
                closed.push_back(node);
                continue;
            }
            if(!exist_same_parent_pair){
                if(node->parent == nullptr){
                    min_g2[node->id][node->t].push_back(std::make_pair(-1, node->g.at(1)));
                }else{
                    min_g2[node->id][node->t].push_back(std::make_pair(node->parent->id, node->g.at(1)));
                }
            }
        }
        
        num_expansion += 1;

        if (node->id == target) {
            solutions.push_back(node);
            log_solution(node);
            continue;
        }
        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            std::vector<size_t> next_g = {node->g[0]+p_edge->cost[0], node->g[1]+p_edge->cost[1]};
            if(if_turn){
                if(node->parent == nullptr){
                    if(next_id != node->id){
                        next_g.at(turn_mode) += turn_cost;
                    }
                }else{
                    int x0 = id2coord[node->parent->id].at(0);
                    int y0 = id2coord[node->parent->id].at(1);
                    int x1 = id2coord[node->id].at(0);
                    int y1 = id2coord[node->id].at(1);
                    int x2 = id2coord[next_id].at(0);
                    int y2 = id2coord[next_id].at(1);
                    if(x1-x0 != x2-x1 || y1-y0 != y2-y1){
                        next_g.at(turn_mode) += turn_cost;
                    }
                }
            }
            auto next_h = heuristic(next_id, node->id, if_turn);
            size_t next_t = node->t + 1;

            // Dominance check
            if(next_id == target){
                if ((1+this->eps_prune[1])*(next_g[1]+next_h[1]) >= min_g2_target){
                    continue;
                }
            }else{
                bool if_dominated = false;
                if(((1+this->eps_prune[1])*(next_g[1]+next_h[1])) >= min_g2_target){
                    if_dominated = true;
                }else if(min_g2[next_id].count(next_t)){
                    for(auto& ele: min_g2[next_id][next_t]){
                        if(if_turn && !(ele.first == -1 && node == nullptr)){
                            if(ele.first != node->id){
                                continue;
                            }
                        }
                        if(next_g.at(1) >= ele.second){
                            if_dominated = true;
                            break;
                        }
                    }
                }
                if (if_dominated) {
                    continue;
                }
            }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            // next = std::make_shared<Node>(next_id, next_g, next_h, next_t, node);
            next = std::make_shared<Node>(next_id, next_g, next_h, next_t, node);
            
            // constraint check
            if(is_constraint(next, vertex_constraints, edge_constraints)){
                continue;
            }
            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);

            closed.push_back(node);
        }  
    }
    std::cout << "Node Expansion = " << num_expansion << std::endl;

    for(size_t i = 0; i < solutions.size(); i++){
        NodePtr pointer = solutions.at(i);
        std::stack<size_t>  id_stack;
        std::vector<size_t> id_vector;
        while(pointer != nullptr){
            id_stack.push(pointer->id);
            pointer = pointer->parent;
        }
        while(!id_stack.empty()){
            id_vector.push_back(id_stack.top());
            id_stack.pop();
        }
        solution_ids.insert(std::make_pair(i, id_vector));
        // solution_apex_costs.insert(std::make_pair(i, solutions.at(i)->g));
        solution_real_costs.insert(std::make_pair(i, solutions.at(i)->g));
    }

    this->end_logging(solutions, true);
    // std::cout << "here";
    // getchar();
}


inline bool is_dominated(SolutionSet & solutions, NodePtr node, Pair<double> eps = {0,0}){
    for (auto sol: solutions){
        if (sol->g[0] <= (1 + eps[0]) * node->g[0] + node->h[0] &&
            sol->g[1] <= (1 + eps[1]) * node->g[1] + node->h[1]
            ){
            return true;
        }
    }
    return false;
}



void BOAStar::log_solution(NodePtr node){
    solution_log.push_back({std::clock() - start_time, node});
}
