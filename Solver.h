#include <iostream>
#include <vector>
#include <algorithm>

#include "Definitions.h"
#include <LowLevel/Utils/MapQueue.h>
#include <HighLevel/ConflictResolver.h>
#include <LowLevel/RunApex.h>

class Solver
{
public:
    Solver(){};
    void init(size_t graph_size, int agent_num, Algorithm algorithm, MergingStrategy& ms, bool if_eager, int dim, int turn_dim, int turn_cost, int solution_num, double eps, double eps_hm, int time_limit);
// //  DomPrune
//     //  for joint_path_list
//     bool DomPrune(std::vector<CostVector>& solution_apexs, std::vector<CostVector>& solution_costs, std::list<JointPathPair>& joint_path_list, double eps, int& DomPruneNum);
//     //  for single joint path
//     bool DomPrune(std::vector<CostVector>& solution_apex_costs, std::vector<CostVector>& solution_costs, CostVector& apex_cost, CostVector& real_cost);


// A*pex version
    void NonDomJointPath(HighLevelNodePtr node, MergingStrategy ms, double eps, int agent_id=-1);
    void NonDomVec(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, std::vector<std::vector<size_t>>& ids_vector, 
        std::vector<int>& conflict_nums_vector, MergingStrategy ms, double eps);
    //  return if can merge
    bool HighLevelMerge(std::pair<CostVector, int>& existing_path, std::pair<CostVector, int>& new_path, CostVector& real_cost1, CostVector& real_cost2, 
        int conflict_num1, int conflict_num2, MergingStrategy ms, double eps);

//  BOA* and NAMOA* version
    void NonDomJointPath(HighLevelNodePtr node);
    void NonDomVec(std::list<JointPathPair>& joint_path_list);

    void calculate_CAT(HighLevelNodePtr, VertexCAT& vertex_cat, EdgeCAT& edge_cat, int agent_id);

    std::tuple<double, double, double, int, int> search(std::vector<Edge>& edges, std::vector<std::pair<size_t, size_t>>& start_end, HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs, LoggerPtr& logger);

// A*pex with dibersity
    void NonDomJointPath(HighLevelNodePtr node, int solution_num, double max_eps=INT_MAX);
    void MergeBySmallestEps(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, int solution_num, double max_eps=INT_MAX);
    // double CD(CostVector& a, CostVector& b, std::vector<double> box_len);
    void MergeBySmallestEps(std::vector<CostVector>& apex_vectors, std::vector<CostVector>& real_costs_vector, int solution_num, double max_eps=INT_MAX);
    // void MergeByDiv(std::vector<CostVector>& apex_cost, std::vector<CostVector>& real_cost, int solution_num, double max_eps=INT_MAX);

protected:
    size_t GRAPH_SIZE;
    int AGENT_NUM;
    int DIM;
    Algorithm ALGORITHM;
    LSolver LSOLVER;
    MergingStrategy MS;
    double EPS;
    bool EAGER;
    int TIME_LIMIT;

    int SOLUTION_NUM;
    int TURN_DIM;
    int TURN_COST;

    MergingStrategy DEFAULT_MS = MergingStrategy::MORE_SLACK;
};