#ifndef BI_CRITERIA_BOA_STAR_H
#define BI_CRITERIA_BOA_STAR_H

#include <vector>
#include "Utils/Definitions.h"
#include "Utils/Logger.h"
#include "AbstractSolver.h"


class BOAStar: public AbstractSolver {
protected:
    std::clock_t start_time;

    std::vector<std::pair<std::clock_t, NodePtr>> solution_log;
    void log_solution(NodePtr);


public:
    virtual std::string get_solver_name() {return "BOA*"; }

    BOAStar(const AdjacencyMatrix &adj_matrix, Pair<double> eps, int turn_mode, int turn_cost, const LoggerPtr logger=nullptr);

    void operator()(PathSet& solution_ids, CostSet& solution_apex_costs, CostSet& solution_real_costs, 
        size_t source, size_t target, Heuristic &heuristic, VertexConstraint& vertex_constraints, 
        EdgeConstraint& edge_constraints, unsigned int time_limit, CAT& cat, std::unordered_map<int, int>& conflict_num_map);

    std::vector<std::pair<std::clock_t, NodePtr>> get_sol_log(){return solution_log;}
};




#endif //BI_CRITERIA_BOA_STAR_H
