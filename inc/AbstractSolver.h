#pragma once

#include "Utils/Definitions.h"
#include "Utils/Logger.h"


class AbstractSolver {
protected:
    const AdjacencyMatrix   &adj_matrix;
    // Pair<double>            eps;
    EPS eps_prune;
    EPS eps_merge;

    size_t num_expansion = 0;
    size_t num_generation= 0;

    virtual void init_search(){
        num_expansion = 0;
        num_generation = 0;
    }

    const LoggerPtr         logger;
    void start_logging(size_t source, size_t target);
    
    void end_logging(SolutionSet &solutions, bool succ);


public:
    virtual std::string get_solver_name() = 0;

    size_t get_num_expansion(){return num_expansion;}
    size_t get_num_generation(){return num_generation;}

    virtual void operator()(PathSet& solution_ids, CostSet& solution_apex_costs, CostSet& solution_real_costs, 
        size_t source, size_t target, Heuristic &heuristic, VertexConstraint& vertex_constraints, 
        EdgeConstraint& edge_constrains, unsigned int time_limit, CAT& cat) = 0;
        
    bool is_constraint(NodePtr node, VertexConstraint& vertex_constraints, EdgeConstraint& edge_constraints);

    
    AbstractSolver(const AdjacencyMatrix &adj_matrix, EPS eps_prune, const LoggerPtr logger): adj_matrix(adj_matrix), eps_prune(eps_prune), logger(logger) {}
    AbstractSolver(const AdjacencyMatrix &adj_matrix, EPS eps_merge, EPS eps_prune, const LoggerPtr logger): adj_matrix(adj_matrix), eps_merge(eps_merge), eps_prune(eps_prune), logger(logger) {}
    virtual ~AbstractSolver(){}
};
