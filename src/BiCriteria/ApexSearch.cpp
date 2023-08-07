#include <memory>
#include <vector>
#include <stack>

#include <iostream>

#include "ApexSearch.h"


ApexSearch::ApexSearch(const AdjacencyMatrix &adj_matrix, EPS eps_merge, EPS eps_prune, const LoggerPtr logger) :
    AbstractSolver(adj_matrix, eps_merge, eps_prune, logger),
    num_of_objectives(adj_matrix.get_num_of_objectives())
{
    expanded.resize(this->adj_matrix.size()+1);
}


void ApexSearch::insert(ApexPathPairPtr &ap, APQueue &queue) {
    std::list<ApexPathPairPtr> &relevant_aps = queue.get_open(ap->id);
    for (auto existing_ap = relevant_aps.begin(); existing_ap != relevant_aps.end(); ++existing_ap) {
        if ((*existing_ap)->is_active == false) {
            continue;
        }
        if (ap->update_nodes_by_merge_if_bounded(*existing_ap, this->eps_merge, ms) == true) {
            // pp and existing_pp were merged successfuly into pp
            // std::cout << "merge!" << std::endl;
            if ((ap-> apex!= (*existing_ap)->apex) ||
                (ap-> path_node!= (*existing_ap)->path_node)) {
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
        size_t source, size_t target, Heuristic &heuristic, IndividualConstraintSet& indiv_constraint_set, 
        unsigned int time_limit) 
{
    init_search();

    auto start_time = std::clock();

    if (num_of_objectives == 2){
        local_dom_checker = std::make_unique<LocalCheck>(eps_merge, this->adj_matrix.size());
        solution_dom_checker = std::make_unique<SolutionCheck>(eps_prune);
    }else{
        local_dom_checker = std::make_unique<LocalCheckLinear>(eps_merge, this->adj_matrix.size());
        solution_dom_checker = std::make_unique<SolutionCheckLinear>(eps_prune);
    }


    this->start_logging(source, target);

    SolutionSet solutions;
    ApexPathSolutionSet ap_solutions;
    ApexPathPairPtr   ap;
    ApexPathPairPtr   next_ap;

    // Saving all the unused PathPairPtrs in a vector improves performace for some reason
    // std::vector<ApexPathPairPtr> closed;

    // Vector to hold mininum cost of 2nd criteria per node
    // std::vector<size_t> min_g2(this->adj_matrix.size()+1, MAX_COST);
    
    // Init open heap
    APQueue open(this->adj_matrix.size()+1);

    NodePtr source_node = std::make_shared<Node>(source, std::vector<size_t>(num_of_objectives, 0), heuristic(source), 0, nullptr);
    ap = std::make_shared<ApexPathPair>(source_node, source_node, heuristic);
    open.insert(ap);

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
        num_generation +=1;

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

            next_ap = std::make_shared<ApexPathPair>(ap, *p_edge);
            
            // Constraint check
            if(is_constraint(next_ap->path_node, indiv_constraint_set)){
                continue;
            }

            // Dominance check
            // if ((((1+this->eps[1])*(bottom_right_next_g[1]+next_h[1])) >= min_g2[target]) ||
            //     (bottom_right_next_g[1] >= min_g2[next_id])) {
            if (is_dominated(next_ap, target)){
                continue;
            }

            // If not dominated extend path pair and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            // std::cout <<"generate node on " << next_ap->id << std::endl;
            this->insert(next_ap, open);
            // closed.push_back(pp);
        }
        // next_ap = std::make_shared<ApexPathPair>(ap, Edge(ap->id, ap->id, std::vector<size_t>({0, 0})));    // remain place action
        // this->insert(next_ap, open);
    }

    // Pair solutions is used only for logging, as we need both the solutions for testing reasons
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
    }
    
    this->end_logging(solutions, true);
}


std::string ApexSearch::get_solver_name() {
    std::string alg_variant;
    if (ms == MergeStrategy::SMALLER_G2){
        alg_variant ="-s2";
    } else if ( ms == MergeStrategy::SMALLER_G2_FIRST){
        alg_variant ="-s2f";
    } else if (ms == MergeStrategy::RANDOM){
        alg_variant ="-r";
    } else if (ms == MergeStrategy::MORE_SLACK){
        alg_variant ="-ms";
    } else if (ms == MergeStrategy::REVERSE_LEX){
        alg_variant ="-rl";
    }
    return "Apex" + alg_variant;
}


void ApexSearch::init_search(){
    AbstractSolver::init_search();
    expanded.clear();
    expanded.resize(this->adj_matrix.size()+1);
}