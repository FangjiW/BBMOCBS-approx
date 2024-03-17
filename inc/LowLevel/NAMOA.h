#pragma once

#include <vector>
#include "Definitions.h"
#include "LowLevel/Utils/Logger.h"
#include "LowLevel/AbstractSolver.h"


class NAMOAdr: public AbstractSolver {
protected:

public:

    NAMOAdr(const AdjacencyMatrix &adj_matrix, EPS eps, int turn_mode, int turn_cost, const LoggerPtr logger=nullptr):     AbstractSolver(adj_matrix, eps, turn_mode, turn_cost, logger) {}

    virtual std::string get_solver_name() {return "NAMOAdr"; }

    void operator()(PathSet& solution_ids, CostSet& solution_apex_costs, CostSet& solution_real_costs, 
        size_t source, size_t target, Heuristic &heuristic, VertexConstraint& vertex_constraints, EdgeConstraint& edge_constraints,
        unsigned int time_limit, VertexCAT& vertex_cat, EdgeCAT& edge_cat, std::unordered_map<int, int>& conflict_num_map);

};


