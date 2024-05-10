#pragma once


#include "Definitions.h"
#include "HighLevel/HighLevelSolver.h"
#include "LowLevel/Utils/MapQueue.h"
#include "HighLevel/ConflictResolver.h"
#include "LowLevel/RunApex.h"



class pexSolver: public HighLevelSolver
{
public:
    pexSolver(size_t graph_size, int agent_num, Algorithm algorithm, bool if_eager, int dim, int turn_dim, int turn_cost, int time_limit)
    : HighLevelSolver(graph_size, agent_num, algorithm, if_eager, dim, turn_dim, turn_cost, time_limit){}
    void set_merging_strategy(MergingStrategy ms){MS = ms;}
    OutputTuple run(std::vector<Edge>& edges, std::vector<std::pair<size_t, size_t>>& start_end, HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs, LoggerPtr& logger) override;
    void set_eps(double eps){EPS = eps;}

protected:
    MergingStrategy DEFAULT_MS = MergingStrategy::MORE_SLACK;
    MergingStrategy MS;
    double EPS;


    void MergeJointPaths(HighLevelNodePtr node, MergingStrategy ms, double eps, int ignored_agent=-1);
    void NonDomVec(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, std::vector<std::vector<size_t>>& ids_vector, 
        std::vector<int>& conflict_nums_vector, MergingStrategy ms, double eps);
    bool HighLevelMerge(std::pair<CostVector, int>& existing_path, std::pair<CostVector, int>& new_path, CostVector& real_cost1, 
        CostVector& real_cost2, int conflict_num1, int conflict_num2, MergingStrategy ms, double eps);
    void PruneApproxDom(std::list<JointPathPair>& jp_list, std::vector<CostVector>& SOLUTIONS_apex, std::vector<CostVector>& SOLUTIONS_cost, double eps);
    void AddSolution(CostVector& apex, CostVector& cost, std::vector<std::vector<size_t>>& waypoints, std::vector<CostVector>& SOLUTIONS_apex, std::vector<CostVector>& SOLUTIONS_cost, HSolutionID& SOLUTIONS_waypoints);
    void EagerSolutionUpdate(HighLevelNodePtr node, std::vector<CostVector>& SOLUTIONS_apex, std::vector<CostVector>& SOLUTIONS_cost, HSolutionID& SOLUTIONS_waypoints);
    void calculate_CAT(HighLevelNodePtr node, VertexCAT& vertex_cat, EdgeCAT& edge_cat, int agent_id);
    void post_process(std::vector<CostVector>& SOLUTIONS_apex, std::vector<CostVector>& SOLUTIONS_cost);
};