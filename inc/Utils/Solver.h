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
    bool DomPrune(std::vector<CostVector>& solution_costs, std::list<JointPathPair>& joint_path_list, double eps, int& DomPruneNum);
    //  for single joint path
    bool DomPrune(std::vector<CostVector>& solution_costs, CostVector& real_cost, double eps);

//  add constraint
    //  vertex constraint
    void add_constraint(std::vector<VertexConstraint>& constraints, size_t agent_id, size_t node_id, size_t time);
    //  edge constraint
    void add_constraint(std::vector<EdgeConstraint>& edge_constraints, size_t agent_id, size_t source, size_t target, size_t time);

// A*pex version
    void NonDomJointPath(HighLevelNodePtr node, MergeStrategy ms, double eps, int agent_id=-1);
    void NonDomVec(std::list<JointPathTuple>& joint_path_vector, MergeStrategy ms, double eps);
    //  return if can merge
    bool HighLevelMerge(JointPathTuple& existing_path, JointPathTuple& new_path, MergeStrategy ms, double eps);

//  BOA* and NAMOA* version
    void NonDomJointPath(HighLevelNodePtr node);
    void NonDomVec(std::list<JointPathPair>& joint_path_list);

    void calculateCAT(HighLevelNodePtr, CAT& cat, int agent_id);

    std::tuple<double, double, double, int, int> search(size_t graph_size, std::vector<Edge>& edges, boost::program_options::variables_map& vm, 
        std::vector<std::pair<size_t, size_t>>& start_end, MergeStrategy& ms, LoggerPtr& logger, 
        HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs, std::ofstream& output_file);
};