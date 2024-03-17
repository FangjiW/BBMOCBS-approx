#pragma once

#include "Definitions.h"
#include "HighLevel/HighLevelSolver.h"
#include "LowLevel/Utils/MapQueue.h"
#include "HighLevel/ConflictResolver.h"
#include "LowLevel/RunApex.h"



class kSolver: public HighLevelSolver
{
protected:
    int SOLUTION_NUM;
    MergingStrategy DEFAULT_MS = MergingStrategy::MORE_SLACK;
    MergingStrategy MS;
    double EPS = 0;


    void MergeJointPaths(HighLevelNodePtr node, int solution_num, double max_eps=INT_MAX);
    void NonDomVec(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, std::vector<std::vector<size_t>>& ids_vector, 
        std::vector<int>& conflict_nums_vector, MergingStrategy ms, double eps);
    void MergeUntil(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, int solution_num, double max_eps=INT_MAX);
    void MergeUntil(std::vector<CostVector>& apex_vectors, std::vector<CostVector>& real_costs_vector, int solution_num, double max_eps=INT_MAX);
    void calculate_CAT(HighLevelNodePtr node, VertexCAT& vertex_cat, EdgeCAT& edge_cat, int agent_id);

    
public:
    kSolver(size_t graph_size, int agent_num, Algorithm algorithm, bool if_eager, int dim, int turn_dim, int turn_cost, int time_limit)
    : HighLevelSolver(graph_size, agent_num, algorithm, if_eager, dim, turn_dim, turn_cost, time_limit){}
    void set_merging_strategy(MergingStrategy ms){MS = ms;}
    void set_solution_num(int solution_num){SOLUTION_NUM = solution_num;}
    OutputTuple run(std::vector<Edge>& edges, std::vector<std::pair<size_t, size_t>>& start_end, HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs, LoggerPtr& logger) override;
};