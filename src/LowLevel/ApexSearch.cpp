#include <memory>
#include <vector>
#include <stack>

#include <iostream>

#include "LowLevel/ApexSearch.h"


ApexSearch::ApexSearch(const AdjacencyMatrix &adj_matrix, EPS eps_merge, EPS eps_prune, int turn_mode, int turn_cost, const LoggerPtr logger) :
    AbstractSolver(adj_matrix, eps_merge, eps_prune, turn_mode, turn_cost, logger),
    num_of_objectives(adj_matrix.get_num_of_objectives())
{
    expanded.resize(this->adj_matrix.size()+1);
}


void ApexSearch::insert(ApexPathPairPtr &ap, APQueue &queue) {
    std::list<ApexPathPairPtr> &relevant_aps = queue.get_open(ap->id, ap->path_node->t);
    for (auto existing_ap = relevant_aps.begin(); existing_ap != relevant_aps.end(); ++existing_ap) {
        if ((*existing_ap)->is_active == false) {
            continue;
        }
        if(turn_mode != -1 && !same_orientation(ap->path_node, (*existing_ap)->path_node)){
            continue;
        }
        if (ap->update_nodes_by_merge_if_bounded(*existing_ap, this->eps_merge, ms) == true) {
            // pp and existing_pp were merged successfuly into pp
            // std::cout << "merge!" << std::endl;
            if ((ap-> apex!= (*existing_ap)->apex) || (ap-> path_node!= (*existing_ap)->path_node)) {
                // If merged_pp == existing_pp we avoid inserting it to keep the queue as small as possible.
                // existing_pp is deactivated and not removed to avoid searching through the heap
                // (it will be removed on pop and ignored)
                (*existing_ap)->is_active = false;
                queue.insert(ap);
            }
            // both apex and path_node are equal -> ap is dominated
            return;
        }
    }
    queue.insert(ap);
}


void ApexSearch::merge_to_solutions(const ApexPathPairPtr &ap, ApexPathSolutionSet &solutions) {
    for (auto existing_solution = solutions.begin(); existing_solution != solutions.end(); ++existing_solution) {
        if ((*existing_solution)->update_nodes_by_merge_if_bounded(ap, this->eps_prune, ms) == true) {
            solution_dom_checker->add_node(*existing_solution);
            return;
        }
    }
    solutions.push_back(ap);
    // std::cout << "update solution checker" << std::endl;
    solution_dom_checker->add_node(ap);
}


bool ApexSearch::is_dominated(ApexPathPairPtr ap, size_t target){
    if(ap->id != target){
        if (local_dom_checker->is_dominated(ap)){
            return true;
        }
    }
    return solution_dom_checker->is_dominated(ap);
}


void ApexSearch::operator()(PathSet& solution_ids, CostSet& solution_apex_costs, CostSet& solution_real_costs, 
        size_t source, size_t target, Heuristic &heuristic, VertexConstraint& vertex_constraints, EdgeConstraint& edge_constraints,
        unsigned int time_limit, VertexCAT& vertex_cat, EdgeCAT& edge_cat, std::unordered_map<int, int>& conflict_num_map) 
{   
    double time = 0;
    double time2 = 0;
    double time3 = 0;
    init_search();
    auto start_time = std::clock();

    bool if_turn = turn_mode == -1 ? false : true;
    
    local_dom_checker = std::make_unique<LocalCheckLinear>(eps_merge, this->adj_matrix.size(), if_turn);
    solution_dom_checker = std::make_unique<SolutionCheckLinear>(eps_prune);

    this->start_logging(source, target);

    SolutionSet solutions;
    ApexPathSolutionSet ap_solutions;
    ApexPathPairPtr   ap;
    ApexPathPairPtr   next_ap;

    // Init open heap
    APQueue open(this->adj_matrix.size()+1);

    NodePtr source_node = std::make_shared<Node>(source, std::vector<size_t>(num_of_objectives, 0), heuristic(source, -1, if_turn), 0, 0, nullptr);
    ap = std::make_shared<ApexPathPair>(source_node, source_node, heuristic);
    open.insert(ap);

// int i = 0 ;
    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            for (auto solution = ap_solutions.begin(); solution != ap_solutions.end(); ++solution) {
                solutions.push_back((*solution)->path_node);
            }
            this->end_logging(solutions, false);
            break;
        }
        // Pop min from queue and process
        ap = open.pop();

        // Optimization: PathPairs are being deactivated instead of being removed so we skip them.
        if (ap->is_active == false) {
            continue;
        }

        // Dominance check
        if (is_dominated(ap, target)){
            continue;
        }
        
        if(ap->id != target){
            //  min_g2[ap->id] = ap->bottom_right->g[1];
            local_dom_checker->add_node(ap);
        }

        num_expansion += 1;

        expanded[ap->id].push_back(ap);

        if (ap->id == target) {
            this->merge_to_solutions(ap, ap_solutions);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[ap->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            // Prepare extension of path pair
            next_ap = std::make_shared<ApexPathPair>(ap, *p_edge, turn_mode, turn_cost);

            if(ap->apex->f.at(0) > next_ap->apex->f.at(0)){
                std::cout << "wrong"; getchar();
            }
            
            // Constraint check
            if(is_constraint(next_ap->path_node, vertex_constraints, edge_constraints)){
                continue;
            }

            // Dominance check
            if (is_dominated(next_ap, target)){
                continue;
            }

            if(vertex_cat.count(next_ap->path_node->t)){
                next_ap->path_node->conflict_num += vertex_cat.at(next_ap->path_node->t).at(next_ap->id);
            }
            if(edge_cat.count(ap->path_node->t)){
                if(edge_cat.at(ap->path_node->t).at(ap->id).count(next_ap->path_node->id)){
                    next_ap->path_node->conflict_num += edge_cat.at(ap->path_node->t).at(ap->id).at(next_ap->path_node->id);
                }
            }
            // for(int ele: cat.at(next_ap->id)){
            //     if(next_ap->path_node->t == ele){
            //         next_ap->path_node->conflict_num ++;
            //     }
            // }

            // If not dominated extend path pair and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            // std::cout <<"generate node on " << next_ap->id << std::endl;
            this->insert(next_ap, open);
            // closed.push_back(pp);
        }
    }

    // Pair solutions is used only for logging, as we need both the solutions for testing reasons
    std::cout << "Node Expansion = " << num_expansion << std::endl;

    solution_ids.clear();
    solution_apex_costs.clear();
    solution_real_costs.clear();
    conflict_num_map.clear();
    for (size_t i = 0; i < ap_solutions.size(); i++){
        solutions.push_back(ap_solutions.at(i)->path_node);
        NodePtr pointer = ap_solutions.at(i)->path_node;
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
        solution_apex_costs.insert(std::make_pair(i, ap_solutions.at(i)->apex->g));
        solution_real_costs.insert(std::make_pair(i, ap_solutions.at(i)->path_node->g));
        conflict_num_map.insert(std::make_pair(i, ap_solutions.at(i)->path_node->conflict_num));
    }
    
    this->end_logging(solutions, true);
}


std::string ApexSearch::get_solver_name() {
    std::string alg_variant;
    if (ms == MergingStrategy::SMALLER_G2){
        alg_variant ="-s2";
    } else if ( ms == MergingStrategy::SMALLER_G2_FIRST){
        alg_variant ="-s2f";
    } else if (ms == MergingStrategy::RANDOM){
        alg_variant ="-r";
    } else if (ms == MergingStrategy::MORE_SLACK){
        alg_variant ="-ms";
    } else if (ms == MergingStrategy::REVERSE_LEX){
        alg_variant ="-rl";
    }
    return "Apex" + alg_variant;
}


void ApexSearch::init_search(){
    AbstractSolver::init_search();
    expanded.clear();
    expanded.resize(this->adj_matrix.size()+1);
}