#include <iostream>
#include <vector>
#include <algorithm>

#include "Utils/Definitions.h"
#include "MapQueue.h"
#include "Utils/ConflictChecker.h"
#include "RunApex.h"

class Solver
{
public:
    Solver(){};
//  DomPrune
    //  for joint_path_list
    bool DomPrune(std::vector<CostVector>& solution_apexs, std::vector<CostVector>& solution_costs, std::list<JointPathPair>& joint_path_list, double eps, int& DomPruneNum);
    //  for single joint path
    bool DomPrune(std::vector<CostVector>& solution_apex_costs, std::vector<CostVector>& solution_costs, CostVector& apex_cost, CostVector& real_cost);

//  add constraint
    //  vertex constraint
    void add_constraint(std::vector<VertexConstraint>& constraints, size_t agent_id, size_t node_id, size_t time);
    //  edge constraint
    void add_constraint(std::vector<EdgeConstraint>& edge_constraints, size_t agent_id, size_t source, size_t target, size_t time);

// A*pex version
    void NonDomJointPath(HighLevelNodePtr node, MergeStrategy ms, double eps, int agent_id=-1);
    void NonDomVec(std::list<std::pair<CostVector, int>>& apex_cost_combos, std::vector<CostVector>& real_costs_vector, std::vector<std::vector<size_t>>& ids_vector, 
        std::vector<int>& conflict_nums_vector, MergeStrategy ms, double eps);
    //  return if can merge
    bool HighLevelMerge(std::pair<CostVector, int>& existing_path, std::pair<CostVector, int>& new_path, CostVector& real_cost1, 
        CostVector& real_cost2, std::vector<size_t>& id1, std::vector<size_t>& id2, int conflict_num1, int conflict_num2, MergeStrategy ms, double eps);

//  BOA* and NAMOA* version
    void NonDomJointPath(HighLevelNodePtr node);
    void NonDomVec(std::list<JointPathPair>& joint_path_list);

    void calculateCAT(HighLevelNodePtr, CAT& cat, int agent_id);

    std::tuple<double, double, double, double, int, int, int, int> search(size_t graph_size, std::vector<Edge>& edges, 
        boost::program_options::variables_map& vm, std::vector<std::pair<size_t, size_t>>& start_end, MergeStrategy& ms, LoggerPtr& logger, 
        HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs);

// A*pex with dibersity
    void NonDomJointPath(HighLevelNodePtr node, int solution_num){};
    void MergeBySmallestEps(std::list<JointPathTuple>& joint_path_vector, int solution_num, double max_eps=INT_MAX);
    double CD(CostVector& a, CostVector& b, std::vector<double> box_len);
    CostVector vector_min(CostVector& a, CostVector& b);
    double calculate_eps(CostVector& a, CostVector& b);
    void MergeBySmallestEps(std::vector<CostVector>& apex_cost, std::vector<CostVector>& real_cost, int solution_num, double max_eps=INT_MAX);
    void MergeByDiv(std::vector<CostVector>& apex_cost, std::vector<CostVector>& real_cost, int solution_num, double max_eps=INT_MAX);
};