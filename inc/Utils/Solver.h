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
    void init(size_t graph_size, int agent_num, Algorithm algorithm, MergeStrategy& ms, bool if_eager, int dim, int turn_dim, int turn_cost, int solution_num, double eps, double eps_hm, int time_limit);
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
    void NonDomVec(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, std::vector<std::vector<size_t>>& ids_vector, 
        std::vector<int>& conflict_nums_vector, MergeStrategy ms, double eps);
    //  return if can merge
    bool HighLevelMerge(std::pair<CostVector, int>& existing_path, std::pair<CostVector, int>& new_path, CostVector& real_cost1, CostVector& real_cost2, 
        int conflict_num1, int conflict_num2, MergeStrategy ms, double eps);

//  BOA* and NAMOA* version
    void NonDomJointPath(HighLevelNodePtr node);
    void NonDomVec(std::list<JointPathPair>& joint_path_list);

    void calculateCAT(HighLevelNodePtr, VertexCAT& vertex_cat, EdgeCAT& edge_cat, int agent_id);

    std::tuple<double, double, double, int, int> search(std::vector<Edge>& edges, std::vector<std::pair<size_t, size_t>>& start_end, HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs, LoggerPtr& logger);

// A*pex with dibersity
    void NonDomJointPath(HighLevelNodePtr node, int solution_num, double max_eps=INT_MAX);
    void MergeBySmallestEps(std::list<std::pair<CostVector, int>>& apex_idx_combos, std::vector<CostVector>& real_costs_vector, int solution_num, double max_eps=INT_MAX);
    double CD(CostVector& a, CostVector& b, std::vector<double> box_len);
    CostVector vector_min(CostVector& a, CostVector& b);
    double calculate_eps(CostVector& a, CostVector& b);
    void MergeBySmallestEps(std::vector<CostVector>& apex_vectors, std::vector<CostVector>& real_costs_vector, int solution_num, double max_eps=INT_MAX);
    // void MergeByDiv(std::vector<CostVector>& apex_cost, std::vector<CostVector>& real_cost, int solution_num, double max_eps=INT_MAX);

protected:
    size_t _graph_size;
    int _agent_num;
    int _dim;
    Algorithm _algorithm;
    LSolver _l_solver;
    MergeStrategy _ms;
    double _eps;
    double _eps_hm;

    bool _if_eager;

    int _turn_dim;
    int _turn_cost;

    int _solution_num;

    int _time_limit;
};