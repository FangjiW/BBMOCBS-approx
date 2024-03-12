#pragma once
#include "Definitions.h"
#include "LowLevel/Utils/Logger.h"
#include "LowLevel/Utils/MapQueue.h"
#include"LowLevel/DominanceChecker.h"
#include "LowLevel/AbstractSolver.h"

class ApexSearch: public AbstractSolver {
protected:
    size_t num_of_objectives;
    MergingStrategy ms=MergingStrategy::SMALLER_G2;

    std::unique_ptr<DominanceChecker> local_dom_checker;
    std::unique_ptr<DominanceChecker> solution_dom_checker;

    virtual void insert(ApexPathPairPtr &pp, APQueue &queue);
    bool is_dominated(ApexPathPairPtr ap, size_t target);
    void merge_to_solutions(const ApexPathPairPtr &pp, ApexPathSolutionSet &solutions);
    std::vector<std::vector<ApexPathPairPtr>> expanded;
    void init_search();

public:

    virtual std::string get_solver_name();


    void set_merge_strategy(MergingStrategy new_ms){ms = new_ms;}
    ApexSearch(const AdjacencyMatrix &adj_matrix, EPS eps_merge, EPS eps_prune, int turn_mode, int turn_cost, const LoggerPtr logger=nullptr);
    virtual void operator()(PathSet& solution_ids, CostSet& solution_apex_costs, CostSet& solution_real_costs, 
        size_t source, size_t target, Heuristic &heuristic, VertexConstraint& vertex_constraints, EdgeConstraint& edge_constraints,
        unsigned int time_limit, VertexCAT& vertex_cat, EdgeCAT& edge_cat, std::unordered_map<int, int>& conflict_num_map); 
};

