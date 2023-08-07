 #include "NAMOA.h"
#include "Utils/MapQueue.h"
#include <stack>


bool is_dominated_dr(NodePtr node, std::list<NodePtr>& list, EPS eps){
    for (auto& n: list){
        if (is_dominated_dr(node, n, eps)){
            return true;
        }
    }
    return false;
}

void add_node_dr(NodePtr node, std::list<NodePtr>& list){
    for (auto it = list.begin(); it != list.end(); ){
        if (is_dominated_dr((*it), node)){
            it = list.erase(it);
        } else {
            it ++;
        }
    }
    list.push_back(node);
}

bool is_dominated_dr(NodePtr node, std::list<NodePtr>& list){
    for (auto& n: list){
        if (is_dominated_dr(node, n)){
            return true;
        }
    }
    return false;
}


// void add_list(NodePtr node, std::list<NodePtr>& list){
    
// }

void NAMOAdr::operator()(PathSet& solution_ids, CostSet& solution_apex_costs, CostSet& solution_real_costs, 
        size_t source, size_t target, Heuristic &heuristic, IndividualConstraintSet& indiv_constraint_set, 
        unsigned int time_limit) {
    this->start_logging(source, target);
    auto start_time = std::clock();

    // std::list<NodePtr> solution_dr;
    SolutionSet solutions;

    NodePtr node;
    NodePtr next;

    NodeQueue open(this->adj_matrix.size()+1);

    // Saving all the unused NodePtrs in a vector improves performace for some reason
    // std::vector<NodePtr> closed;

    // Vector to hold mininum cost of 2nd criteria per node
    std::vector<std::unordered_map<size_t, std::list<NodePtr>>> closed(this->adj_matrix.size());
    std::list<NodePtr>     closed_target;

    // Init open heap
    // std::vector<NodePtr> open;
    // std::make_heap(open.begin(), open.end(), more_than);

    node = std::make_shared<Node>(source, std::vector<size_t>(adj_matrix.get_num_of_objectives(),0), heuristic(source));
    open.insert(node);

    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            this->end_logging(solutions, false);
            break;
        }
        // Pop min from queue and process
        node = open.pop();
        num_generation +=1;

        if (is_dominated_dr(node, closed_target, eps_prune) ||
            closed[node->id].count(node->t) && is_dominated_dr(node, closed[node->id][node->t])
            ){
            continue;
        }
        if(node->id == target){
            if (is_dominated_dr(node, closed_target, eps_prune)){
                continue;
            }else{
                add_node_dr(node, closed_target);
            }
        }else{
            if(closed[node->id].count(node->t) && is_dominated_dr(node, closed[node->id][node->t])){
                continue;
            }else{
                add_node_dr(node, closed[node->id][node->t]);
            }
        }

        num_expansion += 1;

        if (node->id == target) {
            solutions.push_back(node);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            // std::vector<size_t> next_g = {node->g[0]+p_edge->cost[0], node->g[1]+p_edge->cost[1]};
            std::vector<size_t> next_g(node->g.size());
            for (size_t i = 0; i < next_g.size(); i++){
                next_g[i] = node->g[i] + p_edge->cost[i];
            }
            auto next_h = heuristic(next_id);
            next = std::make_shared<Node>(next_id, next_g, next_h, node->t+1, node);

            // confliction check
            if(is_constraint(next, indiv_constraint_set)){
                continue;
            }

            // Dominance check
            if(next->id == target){
                if (is_dominated_dr(next, closed_target, eps_prune)){
                    continue;
                }
            }else{
                if(closed[next->id].count(next->t) && is_dominated_dr(next, closed[next->id][next->t])){
                    continue;
                }
            }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            open.insert(next);

        }
    }

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

        // std::cout << solutions.at(i)->g.at(0) << ", " << solutions.at(i)->g.at(1) << std::endl;
        solution_ids.insert(std::make_pair(i, id_vector));
        // solution_apex_costs.insert(std::make_pair(i, solutions.at(i)->g));
        solution_real_costs.insert(std::make_pair(i, solutions.at(i)->g));
    }

    this->end_logging(solutions, true);
}
