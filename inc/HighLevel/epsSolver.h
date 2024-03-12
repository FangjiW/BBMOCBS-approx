#pragma once

#include "Definitions.h"
#include "HighLevel/HighLevelSolver.h"
#include "LowLevel/Utils/MapQueue.h"
#include "HighLevel/ConflictResolver.h"
#include "LowLevel/RunApex.h"


class epsSolver: public HighLevelSolver
{
protected:
    double EPS;


    void NonDomJointPath(HighLevelNodePtr node);
    void NonDomVec(std::list<JointPathPair>& jp_list);
    OutputTuple run(std::vector<Edge>& edges, std::vector<std::pair<size_t, size_t>>& start_end, HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs, LoggerPtr& logger) override;

public:
    epsSolver(size_t graph_size, int agent_num, Algorithm algorithm, bool if_eager, int dim, int turn_dim, int turn_cost, int time_limit)
    : HighLevelSolver(graph_size, agent_num, algorithm, if_eager, dim, turn_dim, turn_cost, time_limit){}
    void set_eps(double eps) {EPS = eps;};
};