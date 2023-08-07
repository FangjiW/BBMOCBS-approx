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
    bool DomPrune(std::vector<CostVector> solution_costs, std::list<JointPathPair>& joint_path_list, double eps);

    // A*pex version
    void NonDomJointPath(HighLevelNodePtr node, MergeStrategy ms, double eps);
    void NonDomVec(std::list<JointPathTuple>& joint_path_vector, MergeStrategy ms, double eps);
    // std::vector<JointPathTuple> NonDomVec(std::vector<JointPathTuple>  joint_path_vector);

    //  BOA* and NAMOA* version
    void NonDomJointPath(HighLevelNodePtr node);
    void NonDomVec(std::list<JointPathPair>& joint_path_list);

    // std::vector<JointPathTuple> NonDomVec(std::vector<JointPathTuple>& joint_path_vector, MergeStrategy ms, double eps);
    // void search(size_t graph_size, std::vector<Edge>& edges, boost::program_options::variables_map& vm, 
    //     size_t agent_num, double eps, MergeStrategy& ms, LoggerPtr& logger, HSolutionID& hsolution_ids, 
    //     std::vector<CostVector>& hsolution_costs);
    size_t search(size_t graph_size, std::vector<Edge>& edges, boost::program_options::variables_map& vm, 
        std::vector<std::pair<size_t, size_t>> start_end, MergeStrategy& ms, LoggerPtr& logger, 
        HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs);
    void add_constraint(ConstraintSet& constraints, size_t agent_id, size_t node_id, size_t time);

    //  return if can merge
    bool HighLevelMerge(JointPathTuple& existing_path, JointPathTuple& new_path, MergeStrategy ms, double eps);
};