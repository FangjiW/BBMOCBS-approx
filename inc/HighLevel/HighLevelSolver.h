#pragma once


#include "Definitions.h"
#include "LowLevel/Utils/MapQueue.h"
#include "HighLevel/ConflictResolver.h"
#include "LowLevel/RunApex.h"


class HighLevelSolver 
{
protected:
    size_t GRAPH_SIZE;
    int AGENT_NUM;
    int DIM;
    Algorithm ALGORITHM;
    LSolver LSOLVER;
    bool EAGER;
    int TIME_LIMIT;

    int TURN_DIM;
    int TURN_COST;

    time_t start_time;

    int solution_size; // number of solutions

public:
    HighLevelSolver(size_t graph_size, int agent_num, Algorithm algorithm, bool if_eager, int dim, int turn_dim, int turn_cost, int time_limit);

    virtual OutputTuple run(std::vector<Edge>& edges, std::vector<std::pair<size_t, size_t>>& start_end, HSolutionID& hsolution_ids, std::vector<CostVector>& hsolution_costs, LoggerPtr& logger) = 0;
};