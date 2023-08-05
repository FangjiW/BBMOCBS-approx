#include <memory>
#include <algorithm>
#include <time.h>
#include<stack>

#include "BOAStar.h"

BOAStar::BOAStar(const AdjacencyMatrix &adj_matrix, Pair<double> eps, const LoggerPtr logger) :
    AbstractSolver(adj_matrix, {eps[0], eps[1]}, logger) {}


void BOAStar::operator()(PathSet& solution_ids, CostSet& solution_apex_costs, CostSet& solution_real_costs, 
        size_t source, size_t target, Heuristic &heuristic, IndividualConstraintSet& indiv_constraint_set, 
        unsigned int time_limit)  {
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
    std::vector<std::unordered_map<size_t, size_t>> min_g2(this->adj_matrix.size());

    size_t  min_g2_target = UINT64_MAX;

    // Init open heap
    Node::more_than_full_cost more_than;
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    node = std::make_shared<Node>(source, std::vector<size_t>(2,0), heuristic(source));
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);

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
            // std::cout << "node->f[1] = " << node->f[1] << std::endl;
            // getchar();
            if((1+this->eps[1])*node->f[1] >= min_g2_target){
                continue;
            }else{
                min_g2_target = node->g[1];
                // std::cout << "min_g2_target = " << min_g2_target <<std::endl;
                // getchar();
            }
        }else{
            if ((((1+this->eps[1])*node->f[1]) >= min_g2_target) || 
            min_g2[node->id].count(node->t) && (node->g[1] >= min_g2[node->id][node->t])) {
                closed.push_back(node);
                continue;
            }else{
                min_g2[node->id][node->t] = node->g[1];
            }
        }
        
        num_expansion += 1;


        if (node->id == target) {
            // std::cout << node->g[0] << ", " << node->g[1] << std::endl;
            // getchar();
            solutions.push_back(node);
            log_solution(node);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            std::vector<size_t> next_g = {node->g[0]+p_edge->cost[0], node->g[1]+p_edge->cost[1]};
            auto next_h = heuristic(next_id);
            size_t next_t = node->t + 1;

            // Dominance check
            if(next_id == target){
                // std::cout << next_g[1]+next_h[1] << std::endl;
                // std::cout << next_t;
                // getchar();
                if ((1+this->eps[1])*(next_g[1]+next_h[1]) >= min_g2_target){
                    continue;
                }
            }else{
                if ((((1+this->eps[1])*(next_g[1]+next_h[1])) >= min_g2_target) || 
                min_g2[next_id].count(next_t) && (next_g[1] >= min_g2[next_id][next_t])) {
                    closed.push_back(node);
                    continue;
                }
            }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            // next = std::make_shared<Node>(next_id, next_g, next_h, next_t, node);
            next = std::make_shared<Node>(next_id, next_g, next_h, next_t, node);
            
            // constraint check
            if(is_constraint(next, indiv_constraint_set)){
                continue;
            }

            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);

            closed.push_back(node);
        }
    }
    // std::cout << "hreer";
    // getchar();

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
